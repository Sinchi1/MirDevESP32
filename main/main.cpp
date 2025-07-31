#include "HeaderFiles/main.h"

#include "HeaderFiles/BleModule.h"

extern "C" void app_main(void){
    ProgrammRunner runner = ProgrammRunner();
    runner.run();
    std::cout << "Hello, World!\n";
} 

void ProgrammRunner::run(){
    BleModule test = BleModule();
    test.initBT();
}