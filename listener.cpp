#include <stdio.h>
#include <lcm/lcm-cpp.hpp>

#include "bot_core/pointcloud_t.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const bot_core::pointcloud_t* msg)
        {
            printf("Received message on channel \"%s\":\n", chan.c_str());
			
			printf("Points: %d\n", msg->n_points);
			
        }
};

int main (int argc, char const* argv[])
{
	lcm::LCM lcm_sub;
	if(!lcm_sub.good())
		return 1;
	
	Handler handlerObject;
	lcm_sub.subscribe("DRAKE_POINTCLOUD_TEST", &Handler::handleMessage, &handlerObject);
	
	while(0 == lcm_sub.handle());
	
	return 0;
}
