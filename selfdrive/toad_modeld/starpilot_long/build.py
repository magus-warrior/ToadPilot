"""Rebuild only the isolated C3 StarPilot longitudinal solver."""
import os
from pathlib import Path
import subprocess
import sys


def main():
  repo = Path(__file__).resolve().parents[3]
  root = Path(__file__).resolve().parent
  acados = repo / 'third_party/acados'
  env = dict(os.environ, PYTHONPATH=str(acados) + ':' + str(repo),
             ACADOS_SOURCE_DIR=str(acados),
             ACADOS_PYTHON_INTERFACE_PATH=str(acados / 'acados_template'),
             TERA_PATH=str(acados / 'larch64/t_renderer'))
  subprocess.run([sys.executable, '-m', 'selfdrive.toad_modeld.starpilot_long.mpc'], cwd=repo, env=env, check=True)
  makefile = root / 'c_generated_code/Makefile'
  content = makefile.read_text().replace('$(INCLUDE_PATH)/../interfaces/acados_template/acados_template',
                                       str(acados / 'acados_template'))
  content = content.replace(' \\\n\t-L$(EXTERNAL_DIR) -l$(EXTERNAL_LIB)', '')
  makefile.write_text(content)
  subprocess.run(['make', '-C', str(makefile.parent), 'ocp_cython'], env=env, check=True)


if __name__ == '__main__':
  main()
