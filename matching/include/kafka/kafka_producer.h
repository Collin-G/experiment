#pragma once
#include <librdkafka/rdkafkacpp.h>
#include <string>
#include <stdexcept>
#include <memory>

class KafkaProducer {
public:
    KafkaProducer(const std::string& brokers, const std::string& topic, int shard_id)
        : topic_(topic) , shard_id_ (shard_id){

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
        int partition = shard_id_;
        RdKafka::ErrorCode err = producer_->produce(
            topic_,
            partition,                         // explicit partition
            RdKafka::Producer::RK_MSG_COPY,
            const_cast<char*>(payload.c_str()),
            payload.size(),
            nullptr, 0, 0, nullptr
        );
        if (err != RdKafka::ERR_NO_ERROR) {
            throw std::runtime_error("produce failed: " + RdKafka::err2str(err));
        }
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
    int shard_id_;
    std::unique_ptr<RdKafka::Producer> producer_;
};