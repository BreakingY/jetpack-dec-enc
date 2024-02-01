#include "JetsonEnc.h"
#include <unistd.h>
int main()
{
    JetsonEnc *test = new JetsonEnc(352, 288);
    // TODO
    // test->SetDecCallBack(/**/);
    // while(true){
    //     test->AddFrame(/**/,/**/);

    // }
    delete test;
    return 0;
}