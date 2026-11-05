// Render the existing brand artwork on the device's portrait boot surface.
// The compositor uses portrait coordinates; the mounted screen is landscape.
#include <QGuiApplication>
#include <QImage>
#include <QPainter>
#include <QTransform>

int main(int argc, char **argv) {
  QGuiApplication app(argc, argv);
  if (argc != 3) return 2;
  QImage logo(argv[1]);
  if (logo.isNull()) return 3;
  QImage landscape(2160, 1080, QImage::Format_RGB32);
  landscape.fill(Qt::white);
  QPainter painter(&landscape);
  painter.setRenderHint(QPainter::SmoothPixmapTransform);
  const QSize size = logo.size().scaled(1600, 600, Qt::KeepAspectRatio);
  painter.drawImage(QRect(QPoint((2160 - size.width()) / 2, (1080 - size.height()) / 2), size), logo);
  painter.end();
  return landscape.transformed(QTransform().rotate(90)).save(argv[2], "PNG") ? 0 : 4;
}
