#ifndef __APP_TEST_H_
#define __APP_TEST_H_
 
class Test
{
public:
    int x_;
    void init(int x, int y,int z);
    void display(void);
private:
    int y_;
protected:
    int z_;
};

extern Test t;

#endif

