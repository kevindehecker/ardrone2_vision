#include <string.h> 		/* memset */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "defines.h"
#include "exporter.h"
#include <sstream>

#ifdef EXPORT

void Exporter::write(int avgdisp_gt,int avgdisp_gt_stdev, int avgdisp_nn) {
    std::stringstream s;
    s << avgdisp_gt << ";" << avgdisp_gt_stdev << ";" << avgdisp_nn << std::endl;
    myfile << s.str();
}

void Exporter::init() {
    myfile.open ("export.txt");
}

void Exporter::close() {
    myfile.close();
}

#endif
