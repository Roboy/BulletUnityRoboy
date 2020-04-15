```
git clone https://github.com/Roboy/BulletUnityRoboy.git
cd BulletUnityRoboy
git submodule init
git submodule update
```
# pybullet server setup
Get Python >=3.6 and install pybullet.
```
pip3 install --user --upgrade numpy pybullet
```
Run pybullet server either headless by using
```
cd BulletUnityRoboy/python
python runServer.py
```
or for GUI run `App_PhysicsServer_SharedMemory_GUI_vs2010_x64_debug.exe` located in `Assets`.
