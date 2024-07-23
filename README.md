# jetpack-dec-enc
Jetson Video Encoding and Decoding ; Jetson Jetpack5.x视频编解码库
* Jetpack版本： 5.0.2
* Jetpack 5.x编解码是通用的，但是5.0.2编译出来的库不能直接在其他5.x版本上使用，把代码放到目标机器上重新编译即可(不需要替换jetson_multimedia_api头文件)
# 博客
* Jetson视频解码：https://blog.csdn.net/weixin_43147845/article/details/136910084?spm=1001.2014.3001.5502
* Jetson视频编码：https://blog.csdn.net/weixin_43147845/article/details/136914320?spm=1001.2014.3001.5502
# jetson_dec_5.0.2
* 基于Jetpack 5.0.2 jetson_multimedia_api 的视频解码库，支持解码H264、H265,实现过程参考jetson_multimedia_api/samples/02_video_dec_cuda
* 测试程序完善，支持h264裸流、mp4测试文件，h264测试需要修改一下test/video_decode_main.cpp，修改if else分支即可、支持压力测试，可测试jetson解码性能
* 输入：H264/H265  解码器输出：NV12
* 生成动态库libJetsonDec.so,头文件 + libJetsonDec.so 即可作为第三方库使用


# jetson_enc_5.0.2
* 基于Jetpack 5.0.2 jetson_multimedia_api 的视频编码库，实现过程参考jetson_multimedia_api/samples/01_video_encode
* 输入：YUV420P,编码器输出：H264。
* 如需H265可自行修改(JetsonEnc::encode_proc函数/JetsonEnc.cpp)，还是比较简单的。编码参数在void *JetsonEnc::encode_proc(void *arg)中，可根据需求自行修改，修改 / * set your enc arg */注释下的参数即可。
* 生成动态库libJetsonEnc.so,头文件 + libJetsonEnc.so 即可作为第三方库使用


# 编译
1. 解码
* export PATH=$PATH:/usr/local/cuda/bin
* export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
* cd jetson_dec_5.0.2
* tar -zxvf include.tar.gz && tar -zxvf common.tar.gz  这两个文件夹在windows上打开有问题，所以我以压缩包的形式传上来的，需要在linux下解压。可在/usr/src/jetson_multimedia_api和/usr/src/jetson_multimedia_api/samples/中可以找到
* mkdir build
* cd build
* cmake ..
* make -j4
* 测试 ./demo ../../test_video/test_1280x720_v.mp4 1

2. 编码
* export PATH=$PATH:/usr/local/cuda/bin
* export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
* cd jetson_enc_5.0.2
* tar -zxvf include.tar.gz && tar -zxvf common.tar.gz
* mkdir build
* cd build
* cmake ..
* make -j4
* 测试 ./demo ../../test_video/352_288.yuv 352 288 output.h264

# 温馨小提示
* jetson CPU内存和GPU显存是共享的，所以如果使用cuda编程的同学注意，可以把显存地址传给编解码库，但需要使用cudaHostAlloc分配显存，最后一个参数用cudaHostAllocMapped，eg: cudaHostAlloc(&pJetsonAddr,width * heigh * 3,cudaHostAllocMapped)，把pJetsonAddr直接传递给JetsonDec解码类就可以用了，对于JetsonDec这个地址就是v4l2用的内存地址，对于你的应用程序这个地址就是cuda可用的显存地址，这样可节约cudaMemcpy的性能开销。JetsonDec亲测有效，JetsonEnc也是可行的，但是需要修改一下代码(看测试程序就可以知道，应用程序分配的内存是在JetsonEnc内部使用free释放的，但是cudaHostAlloc分配的地址需要使用cudaFreeHost释放才行)，因为并没有这个需求，所以没有实现这个功能。


# 技术交流
* kxsun617@163.com

