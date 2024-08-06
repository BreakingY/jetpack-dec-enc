#include "JetsonEnc.h"
#include <fstream>
#include <iostream>
#include <unistd.h>
char *input;
int width;
int height;
int fps;
char *output;
std::ifstream yuv_file;
std::ofstream output_file;
class EncDataWriter : public JetsonEncListner
{
    void OnJetsonEncData(unsigned char *data, int data_len)
    {
        output_file.write(reinterpret_cast<char *>(data), data_len);
        printf("write bytes :%d \n", data_len);
        return;
    }
};
int main(int argc, char **argv)
{
    if (argc < 6) {
        printf("./demo input width height fps output(H264)\n");
        return -1;
    }
    input = argv[1];
    width = atoi(argv[2]);
    height = atoi(argv[3]);
    fps = atoi(argv[4]);
    output = argv[5];
    // input
    yuv_file.open(input, std::ios::binary);
    if (!yuv_file.is_open()) {
        printf("Error opening the YUV file.\n");
        return 1;
    }
    size_t frame_size = width * height * 3 / 2; // YUV420P
    // output
    output_file.open(output, std::ios::binary | std::ios::app);
    if (!output_file.is_open()) {
        printf("Error opening the output file.\n");
        yuv_file.close();
        return -1;
    }
    // encoder
    JetsonEnc *test = new JetsonEnc(width, height, fps);
    EncDataWriter *writer = new EncDataWriter;
    int frames = 0;
    test->SetDecCallBack(static_cast<JetsonEncListner *>(writer));

    while (!yuv_file.eof()) {
        char *buffer = new char[frame_size];
        yuv_file.read(buffer, frame_size);
        test->AddFrame(buffer, frame_size);
        // JetsonEnc中编码完会释放buffer，此处无需释放，这样做的目的是减少拷贝
        // delete buffer;
        frames++;
        printf("Frame : %d\n", frames);
        if(test->GetQueueSize() >= 5){
            usleep(1000 * 100);
        }
        usleep(1000 * 1000 / fps);
    }
    printf("file over\n");
    // waiting encoder finish
    while(test->GetQueueSize() != 0){
        usleep(1000 * 1000 / fps);
    }
    // release
    delete test;
    delete writer;
    yuv_file.close();
    output_file.close();
    return 0;
}