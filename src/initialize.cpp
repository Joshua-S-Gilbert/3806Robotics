#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

int main(){
    fstream newfile;
    newfile.open("info.txt", ios::in);
    if (!newfile.is_open()){
        //error
    }
    vector<string> params = {};
    string line;
    while (getline(newfile, line)){
        if (line.compare(0, 1, "}") == 0){
            makeObject(params);
            params = {};
            continue;
        }
        params.push_back(line);
    }
}

void makeObject(vector<string> init){
    // make the robot based on init
}