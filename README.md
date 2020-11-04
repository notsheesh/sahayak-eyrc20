# Tasks 
### Setup
1. Clone the repository 
```bash
git clone https://github.com/sciduck/eyrc.git
```
2. Download meshes and models
```bash
cd src && bash setup.sh
```
3. Build workspace 
```bash
cd eyrc
catkin build
```

### Note:
1. You check for changes while you were away 
```bash
git checkout tasks
git pull origin tasks
```
2. Add your changes 
```bash
git checkout tasks
git add <file name>
git commit -am <commit message>
```
3. Push 
```bash
git checkout tasks
git push origin tasks 
```
