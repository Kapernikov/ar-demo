#include <zmq.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream> 
#include <string> 
#include <sstream>
#include <unistd.h>

#if (defined (WIN32))
#include <zhelpers.hpp>
#endif

#define within(num) (int) ((float) num * random () / (RAND_MAX + 1.0))

using namespace std;

int main () {
    unsigned int microseconds = 1000000;

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind("tcp://*:5556");
    //publisher.bind("ipc://weather.ipc");                // Not usable on Windows.

    for (int i=0; i<100; i++ ) {

	float x = 0.0;
    	float y = 0.0 - i;
	float z = 2.2;
	float rx = -1.57;
    	float ry = 0.0;
	float rz = 3.14;

	ostringstream msgJson;
	msgJson << "{\"x\": " << x << ", \"y\": " << y << ", \"z\": " << z << ", \"rx\": " << rx << ", \"ry\": " << ry << ", \"rz\": " << rz << "}" << endl;
	cout << msgJson.str() << endl;

        //  Send message to all subscribers
        zmq::message_t message(msgJson.str().size());
        memcpy (message.data (), (msgJson.str().c_str()), (msgJson.str().size()));
        publisher.send(message);

	usleep(microseconds);

    }
    return 0;
}

