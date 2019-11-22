## C++ version MegaDepth with Point Cloud Visualization
### step 1:
compile libtorch
```
git clone --recursive -b v1.1.0 https://github.com/pytorch/pytorch
cd pytorch && mkdir build && cd build
python ../tools/build_libtorch.py
```
and change torch dir in CMakeLists.txt

### step 2:
download pre-trained megadepth weight
```
http://www.cs.cornell.edu/projects/megadepth/dataset/models/best_generalization_net_G.pth
```
put weight in script

and clone mega depth from 
```
https://github.com/zhengqili/MegaDepth.git
```
put pytorch_DIW_scratch.py in script

### step 3:

run convert.py
```
python3 convert.py
```
note your pytorch should also be 1.1.0
put converted torch jit script in build or data
### step 4
build c++ version mega depth

run
```
./megedepthclient mega_depth_net.pt ../data/demo2.mp4
```
enjoy