"""Exercise real shared-file visibility after both orderly and fatal native exits."""
from pathlib import Path
import signal
import subprocess

import pytest

from bridge_take_journal import HEADER, coverage, finalize


def test_bridge_trace_survives_process_death(tmp_path):
    source = tmp_path / 'writer.cpp'
    source.write_text("""
#include "bridge_take_store.hpp"
#include <csignal>
int main(int argc, char** argv) {
  bridge_take_trace::Store s;
  s.open(argv[1]);
  for (int i=0; i<3; ++i) s.record({2,10+i,20+i,30+i,40+i,.1*i,.02,.03});
  if (argc>2) raise(SIGKILL);
  s.finish();
}
""")
    binary = tmp_path / 'writer'
    subprocess.run(['c++', '-std=c++17', '-O2', '-I', str(Path(__file__).parent),
                    str(source), '-o', str(binary)], check=True)
    clean, killed, corrupt = (tmp_path / n for n in ('clean', 'killed', 'corrupt'))
    for d in (clean, killed, corrupt):
        d.mkdir()
    subprocess.run([binary, clean], check=True)
    assert subprocess.run([binary, killed, 'kill']).returncode == -signal.SIGKILL
    subprocess.run([binary, corrupt], check=True)
    assert finalize(clean)[0].read_bytes() == finalize(killed)[0].read_bytes()
    assert coverage(killed, ['robot2']) == {'robot2': 3}
    with pytest.raises(ValueError, match='Missing DDS'):
        coverage(killed, ['robot1', 'robot2'])
    file, = corrupt.glob('*.journal')
    with file.open('r+b') as out:
        out.seek(HEADER.size)
        out.write(bytes(8))
    with pytest.raises(ValueError, match='Incomplete bridge take'):
        finalize(corrupt)
    assert file.exists() and not list(corrupt.glob('*.csv'))
