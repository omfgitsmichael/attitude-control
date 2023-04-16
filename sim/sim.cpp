#include <iostream>

#include <utils/attitudeUtils.h>
#include <controllers/passivityBasedAdativeControl.h>

int main() {
    attitude::PassivityBasedAdaptiveControl<double>::Params params;
    attitude::PassivityBasedAdaptiveControl<double> controller(params);
    return 0;
}
