#include <UnitBallSampler.hpp>

UnitBallSampler *UnitBallSampler::instance;

UnitBallSampler::UnitBallSampler()
: distribution(-1,1)
{
}

rw::math::Q UnitBallSampler::doSample()
{
    while(1)
    {
        rw::math::Q q_rand = rw::math::Q(6, this->distribution(generator),
                                            this->distribution(generator),
                                            this->distribution(generator),
                                            this->distribution(generator),
                                            this->distribution(generator),
                                            this->distribution(generator));
        if(q_rand.norm2() <= 1.)
        {
            return q_rand;
        }
    }
}
