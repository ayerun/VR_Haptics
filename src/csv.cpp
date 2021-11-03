#include <iostream>
#include <fstream>

int main(int argc, char* argv[]) {
    std::ofstream myfile;
    myfile.open("/test.csv");
    myfile << "1,2,3,4,5,\n";
    myfile << "aflkjlfekj,kiwi,2,\n";
    myfile.close();
    return 0;
}