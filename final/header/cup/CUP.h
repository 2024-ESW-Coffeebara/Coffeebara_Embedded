#ifndef CUP_H
#define CUP_H

class CUP{
    public:
        CUP(int CupSize, int EntranceSize, int existHolder);

        void modifyCupInfo(int CupSize, int EntranceSize, int existHolder);
        int getCupSize();
        int getEntranceSize();
        int getExistHolder();
        
    private:    
        int CupSize;
        int EntranceSize;
        int existHolder;
};

#endif