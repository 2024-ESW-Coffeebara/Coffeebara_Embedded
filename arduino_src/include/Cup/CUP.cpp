#include "Arduino.h"
#include "CUP.h"

CUP::CUP(int CupSize, int EntranceSize, int existHolder){
    this->CupSize = CupSize;
    this->EntranceSize = EntranceSize;
    this->existHolder = existHolder;
}

void CUP::modifyCupInfo(int CupSize, int EntranceSize, int existHolder){
    this->CupSize = CupSize;
    this->EntranceSize = EntranceSize;
    this->existHolder = existHolder;
}

int CUP::getCupSize(){
    return (this->CupSize);
}

int CUP::getEntranceSize(){
    return (this->EntranceSize);
}

int CUP::getExistHolder(){
    return (this->existHolder);
}