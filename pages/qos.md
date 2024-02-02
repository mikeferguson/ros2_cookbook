# QoS
## aka Quality of Service

### Magic Numbers

If you perform `ros2 topic info -v /some/topic` and you examine the QoS settings, you may note that several fields are set to the magic number 2147483651294967295 (or 2,147,483,651,294,967,295). e.g. 

    QoS profile:
        Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
        Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
        Lifespan: 2147483651294967295 nanoseconds
        Deadline: 2147483651294967295 nanoseconds
        Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
        Liveliness lease duration: 2147483651294967295 nanoseconds

What is the significance of this number, which is approximately equal to 68 years? It's not $2^n - 1$ as you might expect for a magic number. However, durations are defined as 

    [builtin_interfaces/Duration]
    # Seconds component, range is valid over any possible int32 value.
    int32 sec

    # Nanoseconds component in the range of [0, 1e9).
    uint32 nanosec

The max value for an `int32` is $2^31 - 1$. The max value for an `uint32` is $2^32 - 1$. (Note: According to the definition, any value for `nanosec` over 1e9 is invalid.)

So the magic number 2147483651294967295 is the number of nanoseconds equivalent to $2^31 -1$ seconds ($2147483647$ seconds) added to $2^32 - 1$ nanoseconds ($4.294967295$ seconds). 