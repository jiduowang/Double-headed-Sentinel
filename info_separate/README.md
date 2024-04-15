## 相机标定信息分离包
该包用于分离相机的标定信息，由于在海康驱动使用的是image_transport::CameraPublisher进行图像的发送以及相机信息获取，该类在源码上限制其发布的相机信息必须以话题名“/camera_info”发出去，因此在多个相机的时候必然会出现话题获取冲突的问题，因此该包主要作用为获取“/camera_info"信息，并根据header里面的frame_id进行区分，并分别以"/camera/camera_info_first"和"/camera/camera_info_second"发送出去。</br>

### info_separate


订阅：
- 所有的相机参数 `/camera_info`

发布：
- 左边的相机参数 `/camera_info_first`
- 右边的相机参数 `/camera_info_second`