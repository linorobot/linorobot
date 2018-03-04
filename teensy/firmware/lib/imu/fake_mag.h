#ifndef _FAKE_MAG_H_
#define _FAKE_MAG_H_


//This a place holder class for devices that does not have a magnetometer

class FakeMag
{
    public:
        FakeMag();
        bool initialize();
        bool testConnection();
        void getHeading(int16_t* mx, int16_t* my, int16_t* mz);
};

FakeMag::FakeMag()
{

}

bool  FakeMag::initialize()
{
    return true;
}

bool  FakeMag::testConnection()
{
    return true;
}

void FakeMag::getHeading(int16_t* mx, int16_t* my, int16_t* mz)
{
    *mx = 0;
    *my = 0;
    *mz = 0;
}

#endif