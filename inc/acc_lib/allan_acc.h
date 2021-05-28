#ifndef AllanAcc_H
#define AllanAcc_H

#include <iostream>
#include <math.h>
#include <vector>
#include "common.h"

namespace imu
{

class AllanAcc
{
    public:
    AllanAcc( std::string name, int maxCluster = 10000 );
    ~AllanAcc( );
    void pushRadPerSec( double data, double time );
    void pushDegreePerSec( double data, double time );
    void pushMPerSec2( double data, double time );
    void calc( );

    std::vector< double > getVariance( ) const;
    std::vector< double > getDeviation( );
    std::vector< double > getTimes( );
    std::vector< int > getFactors( ) const;
    double getFreq( ) const;

    private:
    std::vector< double > calcVariance( double period );

    std::vector< double > calcThetas( const double freq );
    void initStrides( );
    std::vector< double > getLogSpace( float a, float b );
    double getAvgFreq( ) { return 1.0 / getAvgDt( ); }
    double getAvgDt( );
    double getAvgPeriod( ) { return getAvgDt( ); }
    int getFactorsNum( ) { return numFactors; }

    std::string m_name;
    double m_freq;
    int numData;
    std::vector< AccData > m_rawData;
    std::vector< double > m_thetas;
    int numCluster;
    int numFactors;
    std::vector< int > mFactors;

    std::vector< double > mVariance;
};
}

#endif // AllanAcc_H
