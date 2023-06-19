## 飞行汽车场景演示

![首页场景图.gif](/doc_images/intro.gif)

本项目使用 Carla 0.9.14 构建，使用 UnrealEngine 4.62，使用Ubuntu20.04测试。


## 1、基本环境配置
参考 carla 0.9.14 官方文档：[https://carla.readthedocs.io/en/latest/build_linux/](https://carla.readthedocs.io/en/latest/build_linux/)

进入项目根目录，为Python额外安装以下依赖项：
```shell
pip install -r requirements.txt
```

## 2、启动Carla服务器
你可以从以下两种启动方式中任选一种：
### UE编辑器模式(可选)
* 在场景搭建中经常需要调整，这里可以直接使用UE编辑器进行构建场景和基础功能验证。使用方式请参考 [Wiki:UE编辑器模式启动](https://github.com/THU-MIR/Carla-FlyingCar-Exhibition/wiki/%E5%90%AF%E5%8A%A8Carla%E6%9C%8D%E5%8A%A1%E5%99%A8%EF%BC%88UE%E7%BC%96%E8%BE%91%E5%99%A8%E6%A8%A1%E5%BC%8F%EF%BC%89)

### 二进制包模式(可选)
* 受限于产品部署电脑的性能限制，使用UE编辑器启动Carla服务器是一件困难且低效的做法。为了更好的分发，这里通常将其构建好的场景或模型打包成二进制文件再使用。使用方式请参考 [Wiki:二进制包模式启动](https://github.com/THU-MIR/Carla-FlyingCar-Exhibition/wiki/%E5%90%AF%E5%8A%A8Carla%E6%9C%8D%E5%8A%A1%E5%99%A8%EF%BC%88%E4%BA%8C%E8%BF%9B%E5%88%B6%E5%8C%85%E6%A8%A1%E5%BC%8F%EF%BC%89)

加载片刻后，你将看到如下画面：
![Carla服务器启动.png](/doc_images/carla_run.png)

## 3、演示示例
- 本项目在Carla官方示例的基础上额外提供了示例，用以在场景中生成飞行汽车交通流及一辆以第一人称视角，通过键盘操作的飞行汽车。

在运行下列示例前，首先保证 `Carla服务器` 已启动。

进入项目根目录下的 `PythonAPI/examples/`
，再执行：
```shell
# 在场景中生成空中交通流
python3 generate_traffic_air.py

# 在场景中生成地面交通流
python3 generate_traffic_land.py

# 第一人称视角启动飞行汽车（WASD控制朝向和前进后退，方向键上下控制飞行高度）
python3 manual_control_flycar.py
```
