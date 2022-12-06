[<img height="23" src="https://raw.githubusercontent.com/lh9171338/Outline/master/icon.jpg"/>](https://github.com/lh9171338/Outline) 惯导机械编排
===

# 简介
惯导机械编排算法

# 依赖

- eigen3

# 结果

## 定位误差
| method | pos error | vel error | att error |
| :---: | :---: | :---: | :---: |
| baseline | 1.619789e-07 2.319494e-07 1.588303e-02 | 1.673722e-05 1.697424e-05 2.737822e-05 | 1.287195e-07 3.538168e-07 1.419096e-07 |
| 外插 | 1.499726e-07 4.076142e-07 1.855734e-02 | 6.419310e-06 1.593856e-05 3.358931e-05 | 7.655845e-08 5.895792e-08 1.426432e-07 |

# 终端运行

```shell
mkdir build && cd build
cmake ..
make -j8
./IMU
```
