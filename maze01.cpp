
#include <iostream>
#include "path.h"

int main()
{
    int w =1000;
    int h = 1000;
    Path test(w, h);
    //test.show_matrix();
    test.start(1, 0, w-2, h-1);
}
