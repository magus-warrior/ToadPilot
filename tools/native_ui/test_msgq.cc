#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "msgq/msgq.h"
#include "msgq/queue_sizes.h"

static bool fail_mapping = false;
extern "C" void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset) {
  if (fail_mapping) { errno = ENOMEM; return MAP_FAILED; }
  return reinterpret_cast<void *>(syscall(SYS_mmap, addr, length, prot, flags, fd, offset));
}

int main() {
  unsetenv("OPENPILOT_PREFIX");
  setenv("TOADPILOT_BOUNDED_IPC", "1", 1);
  assert(msgq_configured_size("clocks") == DEFAULT_SEGMENT_SIZE);
  std::string prefix = "abrasax-msgq-test-" + std::to_string(getpid());
  std::string directory = "/dev/shm/" + prefix;
  assert(mkdir(directory.c_str(), 0700) == 0);
  setenv("OPENPILOT_PREFIX", prefix.c_str(), 1);
  assert(msgq_configured_size("clocks") == 250 * 1024);
  assert(msgq_configured_size("controlsState") == 2 * 1024 * 1024);
  assert(msgq_configured_size("can") == DEFAULT_SEGMENT_SIZE);
  assert(msgq_configured_size("unknown-service") == DEFAULT_SEGMENT_SIZE);
  msgq_queue_t pub{}, sub{}, bad{};
  assert(msgq_new_queue(&bad, "invalid", 0) == -1 && bad.mmap_p == nullptr);
  fail_mapping = true;
  assert(msgq_new_queue(&bad, "failed", 4096) == -1 && bad.mmap_p == nullptr);
  fail_mapping = false;
  msgq_close_queue(&bad);
  size_t size = msgq_configured_size("clocks");
  assert(msgq_new_queue(&pub, "clocks", size) == 0);
  msgq_init_publisher(&pub);
  assert(msgq_new_queue(&sub, "clocks", size) == 0);
  msgq_init_subscriber(&sub);
  struct stat st{};
  assert(stat((directory + "/clocks").c_str(), &st) == 0);
  assert(static_cast<size_t>(st.st_size) == size + sizeof(msgq_header_t));
  // Exercise many complete ring wraps with exact payload checks.
  for (int i = 0; i < 10000; ++i) {
    char bytes[1024]{};
    memcpy(bytes, &i, sizeof(i));
    msgq_msg_t message{}, received{};
    assert(msgq_msg_init_data(&message, bytes, sizeof(bytes)) == 0);
    assert(msgq_msg_send(&message, &pub) >= 0);
    assert(msgq_msg_recv(&received, &sub) > 0);
    assert(received.size == sizeof(bytes));
    assert(memcmp(received.data, bytes, sizeof(bytes)) == 0);
    msgq_msg_close(&message); msgq_msg_close(&received);
  }
  msgq_close_queue(&pub); msgq_close_queue(&sub);
  unlink((directory + "/clocks").c_str());
  unlink((directory + "/failed").c_str());
  assert(rmdir(directory.c_str()) == 0);
  return 0;
}
