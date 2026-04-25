#pragma once
#include <librdkafka/rdkafkacpp.h>
#include <string>
#include <stdexcept>
#include <memory>

class KafkaProducer {
public:
    KafkaProducer(const std::string& brokers, const std::string& topic)
        : topic_(topic) {

        std::string errstr;

        // conf is just a key-value store of settings
        // unique_ptr so it cleans itself up
        std::unique_ptr<RdKafka::Conf> conf(
            RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL)
        );

        // tell librdkafka where kafka is running
        // bootstrap means "start here, discover the rest of the cluster"
        if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) {
            throw std::runtime_error("bootstrap.servers failed: " + errstr);
        }

        // acks=all means kafka waits for ALL replicas to write the message
        // before confirming — safest option, slight latency cost
        // for match events you want this — losing a match notification is bad
        if (conf->set("acks", "all", errstr) != RdKafka::Conf::CONF_OK) {
            throw std::runtime_error("acks failed: " + errstr);
        }

        // create the actual producer handle
        producer_.reset(RdKafka::Producer::create(conf.get(), errstr));
        if (!producer_) {
            throw std::runtime_error("create producer failed: " + errstr);
        }
    }

    void produce(const std::string& payload) {
        RdKafka::ErrorCode err = producer_->produce(
            topic_,
            RdKafka::Topic::PARTITION_UA,    // UA = unassigned, let kafka pick
            RdKafka::Producer::RK_MSG_COPY,  // kafka makes its own copy of payload
            const_cast<char*>(payload.c_str()),
            payload.size(),
            nullptr,  // no key
            0,
            0,        // timestamp, 0 = use current time
            nullptr   // no opaque pointer
        );

        if (err != RdKafka::ERR_NO_ERROR) {
            throw std::runtime_error("produce failed: " + RdKafka::err2str(err));
        }

        // librdkafka batches messages internally for throughput
        // it doesn't send immediately — it buffers and sends in batches
        // poll(0) gives it a chance to process delivery confirmations
        // without blocking — non-blocking check
        producer_->poll(0);
    }

    void poll(int timeout_ms) {
    producer_->poll(timeout_ms);
}

    void flush() {
        // on shutdown wait up to 5 seconds for all buffered messages to send
        // without this you can lose the last few messages on clean shutdown
        producer_->flush(5000);
    }

private:
    std::string topic_;
    std::unique_ptr<RdKafka::Producer> producer_;
};