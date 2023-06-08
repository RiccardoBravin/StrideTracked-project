#pragma once
#include <cstdarg>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class DecisionTree {
                public:
                    /**
                    * Predict class for features vector
                    */
                    int predict(float *x) {
                        if (x[3] <= 0.593578040599823) {
                            if (x[25] <= 0.008021324872970581) {
                                if (x[14] <= 0.15240517258644104) {
                                    if (x[21] <= 0.5855567157268524) {
                                        if (x[29] <= 0.3288743197917938) {
                                            return 2;
                                        }

                                        else {
                                            return 0;
                                        }
                                    }

                                    else {
                                        return 3;
                                    }
                                }

                                else {
                                    return 3;
                                }
                            }

                            else {
                                if (x[2] <= 0.15240517258644104) {
                                    if (x[20] <= 1.6363502740859985) {
                                        if (x[4] <= -0.248661071062088) {
                                            return 0;
                                        }

                                        else {
                                            return 2;
                                        }
                                    }

                                    else {
                                        return 2;
                                    }
                                }

                                else {
                                    return 3;
                                }
                            }
                        }

                        else {
                            return 1;
                        }
                    }

                protected:
                };
            }
        }
    }