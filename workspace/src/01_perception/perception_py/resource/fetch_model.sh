# may need to run "chmod u+x src/01_perception/perception_py/resource/fetch_model.sh" to handle permission denied error

wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=155teU0MztfP7G3vtXZCIb8rsdSyRL4pr' -O yolov4-tiny-evgrandprix22.tar.gz
tar -C src/01_perception/perception_py/resource/ -xvf yolov4-tiny-evgrandprix22.tar.gz
rm yolov4-tiny-evgrandprix22.tar.gz