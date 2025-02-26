import sys
from pathlib import Path

# Path where `mylib_cpp` Python binding has been generated.
# In this case, `mylib_cpp` is defined in a shared library that looks like:
# mylib_cpp.cpython-<python-version>-<architecture>-<platform>.so
sys.path.append(str(Path.cwd() / "build/install/lib"))

import mylib_cpp


def main():
    print(mylib_cpp.mylib_fn())


if __name__ == "__main__":
    main()
