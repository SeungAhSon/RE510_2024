Dependencies for G2O(UBUNTU)

libsuitesparse-dev
qtdeclarative5-dev
qt5-qmake
libqglviewer-dev


Install command (for Python 2.7)

cd prereq/g2opy
mkdir build
cd build
cmake .. -DPYBIND11_PYTHON_VERSION=2.7
make -j8
cd ..
python setup.py install

(g2opy\python\core\eigen_types.h의 quaterniond::x,y,z,w부분은 일부 환경에서 안되는 듯 하여 주석처리하였으며,
 본 수업 프로그램엔 문제가 없습니다.)

Python dependencies (for Python 2.7)

pip install scipy
