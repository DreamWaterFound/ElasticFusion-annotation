使用CUDA 10.

NVCC的编译选项仅编译了计算能力６.1。有需要可以在CMakeLists.txt中进行相应的改动

不过CUDA10中有些特性已经被deprecated，方便起见屏蔽警告。
