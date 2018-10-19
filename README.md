# Gateway from an e-stop switch sending UDP messages to ROS

This packages will take messages sent by a network-based emergency stop device, like the [esp8266 emergency stop][esp8266_estop] and forward them to a ROS topic, after checking their integrity.

## Status: almost ready for production

We currently only lack the code that goes fetch the keys for checking and signing the heartbeat messages. They are for now hard-coded in the script.

Also, we would like to find a way for more than one emergency stop to be used at the same time, for the same robot.

## Usage

`rosrun estop_gateway node.py` is all you should need for the current version, assuming the development setup that we use. You might just need to change the target IP in the emergency button's settings.

## Short description of the Heartbeat message

The header message is very simple, with only three (used) values: the timestamp of the emitted message, the "count" and the hash of the heartbeat.

Within a given second, the heartbeats must have a strictly increasing value for "count". It can be reset after each new second passes. At all times, the hash of the heartbeat must match with the rest of the message plus a secret key.

<!-- TODO: point to the documentation of the protocol, in the ESP8266 repository -->

> Note that we did not manage to use the "seq" field of the header of the message to store our "count" variable.

## Design questions

Missing feature for the gateway: it needs to check that all previous messages within the limit time were only received once.

### How to handle several estop buttons

I need to decide how to handle several estop buttons. I can basically have one gateway for all switches or one for each and a multiplexer. Likely, I should choose the kind of message that is relayed to the endpoints.

Single gateway brings:

- single point in the system having the emergency stops' keys
- no need for an other node for the multiplexing

Device-specific gateways allow:

- other gateways to be used for other kinds of e-stops
- simpler and less bloated gateway
- no need for sub-process or such to handle several UDP connections
- addition of a multiplexing node that
  - listens to a number of topics, publishing all at a given frequency
  - republishes a topic as long as none of the input pulses has missed more than one pulse

Should we relay the original messages ? Better not, that would prevent us from using more than one estop buttons. No because there is no proper timestamp produced by ESP8266 (resolution too limited, below second too uncertain).
Should we hash the relayed messages ? I would tend to say yes, but optional. We can have two topics generated (on demand) by the gateway or the multiplexer, one with hash and the other without

## Improvements in consideration

### Warn the user of low battery level

It should be this node's job to warn the user, probably through ROS logging messages of WARN level. Additionally, we could consider to have this node publish the battery level(s) on an other topic.

### UDP Transport for the topics

We should setup the transport to UDP for the ROS topics. It's impossible for `rospy`...

This design hides delays in the network. For instance, with a ping of 100ms, the last broadcast would reach destination 100ms too late. Conversely, ping techniques show the latency well. However, we can use timestamp to fix this.

Hashes could be stored in message header, in frameID.

[esp8266_estop]: https://gitlab.inria.fr/resibots/esp8266_estop