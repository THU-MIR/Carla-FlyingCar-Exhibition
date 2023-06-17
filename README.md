## 飞行汽车场景演示

![首页场景图.gif](/doc_images/intro.gif)

本项目使用 Carla 0.9.14 构建，使用 UnrealEngine 4.62，使用Ubuntu20.04测试。


## 1、基本环境配置
参考 carla 0.9.14 官方文档：[https://carla.readthedocs.io/en/latest/build_linux/](https://carla.readthedocs.io/en/latest/build_linux/)

进入项目根目录，为Python额外安装以下依赖项：
```shell
pip install -r requirements.txt
```

## 2、启动虚幻引擎
进入 carla 项目根目录，执行：
```shell
make launch
```
加载片刻后，你将看到如下画面：
![UE启动界面.png](/doc_images/ue_run.png)


## 3、导入飞行汽车模型及动画
- 资源文件一共包含两个文件夹，其中 `Rotationcycle` 是有旋翼桨叶旋转动画的飞行汽车，`Spread` 是带旋翼展开动画的飞行汽车。上述文件已在 UE4.62 版本中编辑好，可以直接进行导入。

1、复制 `Rotationcycle` 文件夹和 `Spread` 文件夹至 `carla/Unreal/CarlaUE4/Content` 目录下以正确加载资源文件。

导入成功后如下图所示，在UE编辑器中内容浏览器下可以看到上述文件夹和对应资源文件：

![导入资源文件夹.png](/doc_images/load_modelfold.png)

2、为飞行汽车添加蓝图和参数，参考 [Wiki](https://github.com/Derkai52/Carla-FlyingCar-Exhibition/wiki)

其中材质渲染及动画部分本示例默认提供，也可以使用Blend或UE自行实现。
除本示例外，可参考官方文档[导入自定义车辆](https://carla.readthedocs.io/en/latest/tuto_A_add_vehicle/#bind-and-model-the-vehicle) 

## 4、启动Carla服务器（UE编辑器模式）
- 在场景搭建中经常需要调整，这里可以直接使用UE编辑器进行构建场景和基础功能验证。

点击UE编辑器上方界面的 `运行` 按钮，稍等片刻，Carla 服务器将启动。

## 5、启动Carla服务器（二进制包模式）
受限于产品部署电脑的性能限制，使用UE编辑器启动Carla服务器是一件困难且低效的做法。为了更好的分发，这里通常将其构建好的场景或模型打包成二进制文件再使用。

进入 carla 项目根目录，执行：
```shell
make package
```

此时 carla 项目根目录下你可以找到新编译的二进制包，默认状态下它位于 `Dist` 文件夹下。执行 `CarlaUE4.sh` 以加载二进制包形式启动 Carla 服务器。

![加载二进制包以启动Carla服务器.png](/doc_images/load_binary_package.png)

- 注意：默认状态下 `make package` 命令为覆盖式生成，即会覆盖上一次已编译好的包，所以建议将编译好的包自行备份。

## 6、演示示例
- 本项目在Carla官方示例的基础上额外提供了 `manual_control_flycar.py` 用以在场景中生成飞行汽车交通流及一辆以第一人称视角，通过键盘操作的飞行汽车。

进入项目根目录下的 `PythonAPI/examples/`
首先保证Carla服务器已启动，再执行：
```shell
# 在场景中生成空中交通流
python3 generate_traffic_air.py

# 在场景中生成地面交通流
python3 generate_traffic_land.py

# 第一人称视角启动飞行汽车（WASD控制朝向和前进后退，方向键上下控制飞行高度）
python3 manual_control_flycar.py
```
