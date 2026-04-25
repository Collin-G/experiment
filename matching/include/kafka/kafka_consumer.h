#pragma once
#include <librdkafka/rdkafkacpp.h>
#include <string>
#include <stdexcept>
#include <memory>
#include <optional>

class KafkaConsumer {
public:
    KafkaConsumer(const std::string& brokers,
                  const std::string& group_id,
                  const std::string& topic) {

        std::string errstr;

        std::unique_ptr<RdKafka::Conf> conf(
            RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL)
        );

        if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) {
            throw std::runtime_error("bootstrap.servers failed: " + errstr);
        }

        // group.id is how kafka tracks your consumer's position in the log
        // if you restart with the same group.id, kafka resumes where you left off
        // if you use a different group.id, kafka starts from scratch
        // for your shard worker use something like "shard-0-worker"
        if (conf->set("group.id", group_id, errstr) != RdKafka::Conf::CONF_OK) {
            throw std::runtime_error("group.id failed: " + errstr);
        }

        // what to do on first run when there's no saved position
        // earliest = start from the very beginning of the log
        // latest = only process new messages from now on
        // earliest is safer — don't miss any commands
        if (conf->set("auto.offset.reset", "earliest", errstr) != RdKafka::Conf::CONF_OK) {
            throw std::runtime_error("auto.offset.reset failed: " + errstr);
        }

        consumer_.reset(RdKafka::KafkaConsumer::create(conf.get(), errstr));
        if (!consumer_) {
            throw std::runtime_error("create consumer failed: " + errstr);
        }

        // subscribe to the topic
        // kafka assigns partitions to this consumer
        // with one partition per topic this is straightforward
        std::vector<std::string> topics = {topic};
        RdKafka::ErrorCode err = consumer_->subscribe(topics);
        if (err != RdKafka::ERR_NO_ERROR) {
            throw std::runtime_error("subscribe failed: " + RdKafka::err2str(err));
        }
    }

    std::optional<std::string> poll(int timeout_ms = 100) {
        // consume blocks until either:
        // 1. a message arrives → returns immediately
        // 2. timeout_ms elapses → returns with timeout error
        // this is NOT busy waiting — the thread actually sleeps
        RdKafka::Message* msg = consumer_->consume(timeout_ms);

        std::optional<std::string> result;

        switch (msg->err()) {
            case RdKafka::ERR_NO_ERROR:
                // real message — extract payload
                result = std::string(
                    static_cast<char*>(msg->payload()),
                    msg->len()
                );
                break;

            case RdKafka::ERR__TIMED_OUT:
                // no message within timeout — normal, not an error
                // result stays nullopt
                break;

            case RdKafka::ERR__PARTITION_EOF:
                // reached end of current messages — normal
                // result stays nullopt
                break;

            default:
                // real error — throw so the main loop can handle it
                delete msg;
                throw std::runtime_error("consume error: " + msg->errstr());
        }

        // must always delete — librdkafka allocates this every call
        delete msg;
        return result;
    }

    ~KafkaConsumer() {
        // cleanly tell kafka we're done
        // without this kafka thinks we crashed and waits for us to reconnect
        consumer_->close();
    }

private:
    std::unique_ptr<RdKafka::KafkaConsumer> consumer_;
};