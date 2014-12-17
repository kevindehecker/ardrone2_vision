#ifndef Exporter_H
#define Exporter_H
#include "defines.h"
#ifdef EXPORT

#include <iostream>
#include <fstream>
#include <sstream>

/*
 * This class exports data to csv txt file, which may be imported in Matlab etc
 *
 */
class Exporter{
private:
    /*  Global variables  */
    std::ofstream myfile;


    /*  Function declarations  */


public:
    /*  Global variables  */


    /*  Function declarations  */
    void close();
    void init();
    void write(int avgdisp_gt,int avgdisp_gt_stdev, int avgdisp_nn);
};
#endif // EXPORT
#endif  /*  Exporter_H  */

