Missing feature for the gateway: it needs to check that all previous messages within the limit time were only received once.

I need to decide how to handle several "remiwitch". I can basically have one gateway for all switches or one for each and a multiplexer. Likelly, I should choose the kind of message that is relayed to the endpoints.

Single gateway brings
- single point in the system having the emergency stops' keys
- no need for an other node for the multiplexing

Device-specific gateways allow
- other gateways to be used for other kinds of e-stops
- simpler and less bloated gateway
- no need for subprocess or such to handle several UDP connections
- addition of a multiplexing node that
  - listens to a number of topics, publishing all at a given frequency
  - republishes a topic as long as none of the input pulses has missed more than one pulse

Should we relay the original messages ? Better not, that would prevent us from using more than one "remiwitch". No because there is no proper timestamp produced by ESP8266 (resolution too limited, bellow second too uncertain).
Should we hash the relayed messages ? I would tend to say yes, but optional. We can have two topics generated (on demand) by the gateway or the multiplexer, one with hash and the other without

We should setup the transport to UDP for the ROS topics. It's impossible for rospy...

This design hides delays in the network. For instance, with a ping of 100ms, the last broadcast would reach destination 100ms too late. Conversely, ping techniques show the latency well. However, we can use timestamp to fix this.

Hashes could be stored in message header, in frameID.