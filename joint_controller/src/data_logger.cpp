#include "joint_controller/data_logger.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace jc {
namespace {
constexpr std::size_t kFileBufferBytes = 1U << 20;
constexpr const char* kStateNames[] = {"INIT", "IDLE", "ARMED", "RUN", "STOP", "FAULT"};
constexpr const char* kFaultNames[] = {
    "NONE", "CONSECUTIVE_READ_FAILURES", "SNAPSHOT_READ_TIMEOUT", "NONFINITE_FEEDBACK",
    "SEMAPHORE_RELEASE_ERROR", "CLOCK_ERROR", "SLEEP_ERROR", "LOGGER_ERROR",
    "TRACKING_ERROR", "TARGET_WRITE_ERROR", "FORCE_SENSOR_INVALID_OR_ZERO_CHANGED", "EXPERIMENT_SAFETY", "EMERGENCY_STOP", "HEARTBEAT_TIMEOUT"};
constexpr const char* kReadNames[] = {
    "OK", "CONTENDED", "INTERRUPTED", "SEMAPHORE_ERROR", "RELEASE_ERROR", "NOT_ATTACHED", "CLOCK_ERROR"};
constexpr const char* kCommandNames[] = {"NONE", "ARM", "RUN", "STOP", "RESET", "MOVE", "FORCE_ZERO", "CONFIGURE", "PRELOAD", "UNLOAD", "BEGIN_LOAD", "COMMIT_LOAD", "RESUME_CSP"};
constexpr const char* kResultNames[] = {
    "NONE", "ACCEPTED", "REJECTED_STATE", "REJECTED_FEEDBACK", "REJECTED_VELOCITY",
    "REJECTED_ARGUMENT"};
constexpr const char* kModeNames[] = {"READ_ONLY_DRY_RUN", "CSP_BUILTIN", "CST_RAW"};
constexpr const char* kPhaseNames[] = {
    "IDLE", "INITIAL_HOLD", "OUTBOUND", "TARGET_HOLD", "RETURN", "FINAL_HOLD",
    "COMPLETE", "STOPPED"};

class CsvRow {
public:
    explicit CsvRow(std::FILE* file) : file_(file) {}
    void text(const char* value) { delimiter(); std::fputs(value, file_); }
    void number(std::uint64_t value) {
        delimiter(); std::fprintf(file_, "%llu", static_cast<unsigned long long>(value));
    }
    void signed_number(std::int64_t value) {
        delimiter(); std::fprintf(file_, "%lld", static_cast<long long>(value));
    }
    void real(double value) { delimiter(); std::fprintf(file_, "%.9g", value); }
    void empty() { delimiter(); }
    void end() { std::fputc('\n', file_); }
private:
    void delimiter() {
        if (!first_) { std::fputc(',', file_); }
        first_ = false;
    }
    std::FILE* file_;
    bool first_{true};
};

void write_header(std::FILE* file) {
    std::fputs("record_time_s,cycle_index,scheduled_wakeup_ns,actual_wakeup_ns,wakeup_jitter_ns,"
               "actual_dt_valid,actual_dt_ns,feedback_read_duration_ns,cycle_execution_ns,"
               "deadline_miss,skipped_before_cycle,skipped_after_cycle,snapshot_read_ok,"
               "snapshot_read_timestamp_ns,snapshot_read_age_ns,snapshot_read_timeout,"
               "source_feedback_freshness,source_feedback_timeout,read_status,read_errno,"
               "feedback_valid,finite_mask,controllable_mask,cycle_valid,state,fault,"
               "run_entry_cycle,control_compute_executed,command,command_result,"
               "last_command_sequence,last_command_received_ns,controller_reset_count,"
               "total_reads,successful_reads,failed_reads,consecutive_failed_reads,"
               "semaphore_contention_count,maximum_read_duration_ns,dropped_record_count,"
               "control_mode,motion_phase,reference_position,reference_velocity,actual_position,"
               "actual_velocity,submitted_csp_position,target_write_ok,target_write_failures,"
               "tracking_error,trajectory_complete,actual_at_target,"
               "force_raw,force_zero,force_external,force_package_no,force_timestamp_ns,force_valid,force_new_sample,force_zeroed,"
               "experiment_id,experiment_stage,experiment_direction,experiment_level,commanded_current_raw,actual_current_raw,actual_current_valid", file);
    for (std::size_t i = 0; i < kJointCount; ++i) {
        if(i==kJ3)std::fputs(",j3_q,j3_v,j3_current_raw_6077,j3_hold_q,j3_pending_effort",file);
        else std::fprintf(file, ",j%zu_q,j%zu_v,j%zu_effort,j%zu_hold_q,j%zu_pending_effort", i, i, i, i, i);
    }
    std::fputs(",force_age_ns,force_sample_sequence,force_session_id,force_zero_version,heartbeat_timestamp_ns,heartbeat_check_timestamp_ns,heartbeat_age_ns\n",file);
}

void write_record(std::FILE* file, const CycleRecord& record,std::uint64_t start) {
    CsvRow row(file);
    row.real(static_cast<double>(record.actual_wakeup_ns-start)*1e-9);
    row.number(record.cycle_index);
    row.number(record.scheduled_wakeup_ns);
    row.number(record.actual_wakeup_ns);
    row.signed_number(record.wakeup_jitter_ns);
    row.number(record.actual_dt_valid);
    row.number(record.actual_dt_ns);
    row.number(record.feedback.snapshot_read_duration_ns);
    row.number(record.cycle_execution_ns);
    row.number(record.deadline_miss);
    row.number(record.skipped_before_cycle);
    row.number(record.skipped_after_cycle);
    row.number(record.feedback.snapshot_read_ok);
    row.number(record.feedback.snapshot_read_timestamp_ns);
    row.number(record.snapshot_age_ns);
    row.number(record.snapshot_read_timeout);
    row.text("UNKNOWN");
    row.text("UNSUPPORTED");
    row.text(kReadNames[static_cast<std::size_t>(record.feedback.read_status)]);
    row.signed_number(record.feedback.error_number);
    row.number(record.feedback.feedback_valid);
    row.number(record.feedback.finite_mask);
    row.number(kControllableMask);
    row.number(record.cycle_valid);
    row.text(kStateNames[static_cast<std::size_t>(record.state)]);
    row.text(kFaultNames[static_cast<std::size_t>(record.fault)]);
    row.number(record.run_entry_cycle);
    row.number(record.control_compute_executed);
    row.text(kCommandNames[static_cast<std::size_t>(record.command)]);
    row.text(kResultNames[static_cast<std::size_t>(record.command_result)]);
    row.number(record.last_command_sequence);
    row.number(record.last_command_received_ns);
    row.number(record.controller_reset_count);
    row.number(record.reads.total_reads);
    row.number(record.reads.successful_reads);
    row.number(record.reads.failed_reads);
    row.number(record.reads.consecutive_failed_reads);
    row.number(record.reads.semaphore_contention_count);
    row.number(record.reads.maximum_read_duration_ns);
    row.number(record.dropped_record_count);
    row.text(kModeNames[static_cast<std::size_t>(record.control_mode)]);
    row.text(kPhaseNames[static_cast<std::size_t>(record.motion_phase)]);
    row.real(record.reference_position);
    row.real(record.reference_velocity);
    if (record.feedback.snapshot_read_ok) {
        row.real(record.feedback.joints[kJ3].position);
        row.real(record.feedback.joints[kJ3].speed);
    } else { row.empty(); row.empty(); }
    row.real(record.submitted_csp_position);
    row.number(record.target_write_ok);
    row.number(record.target_write_failures);
    row.real(record.tracking_error);
    row.number(record.trajectory_complete);
    row.number(record.actual_at_target);
    row.real(record.force.raw);row.real(record.force.zero);row.real(record.force.external);
    row.number(record.force.package_no);row.number(record.force.timestamp_ns);
    row.number(record.force.valid);row.number(record.force.new_sample);row.number(record.force.zeroed);
    row.number(record.experiment_id);row.text(stage_name(record.experiment_stage));
    row.signed_number(record.experiment_direction);row.number(record.experiment_level);
    row.real(record.commanded_current_raw);
    if(record.actual_current_valid)row.real(record.actual_current_raw);else row.empty();
    row.number(record.actual_current_valid);
    for (std::size_t i = 0; i < kJointCount; ++i) {
        if (record.feedback.snapshot_read_ok) {
            row.real(record.feedback.joints[i].position);
            row.real(record.feedback.joints[i].speed);
            row.real(record.feedback.joints[i].effort);
        } else {
            row.empty(); row.empty(); row.empty();
        }
        row.real(record.hold_reference[i]);
        row.real(record.pending_effort[i]);
    }
    if(record.force.timestamp_ns!=0&&record.force_read_timestamp_ns>=record.force.timestamp_ns)
        row.number(record.force_read_timestamp_ns-record.force.timestamp_ns);
    else row.empty();
    row.number(record.force.sample_sequence);row.number(record.force.session_id);row.number(record.force.zero_version);
    row.number(record.heartbeat_timestamp_ns);row.number(record.heartbeat_check_timestamp_ns);
    if(record.heartbeat_timestamp_ns&&record.heartbeat_check_timestamp_ns>=record.heartbeat_timestamp_ns)
        row.number(record.heartbeat_check_timestamp_ns-record.heartbeat_timestamp_ns);
    else row.empty();
    row.end();
}

void write_summary(std::FILE* file, const LoopSummary& summary, const LoopConfig& config,
                   std::uint64_t dropped, std::uint64_t formatted, int io_error) {
    std::fputs("metric,value\n", file);
    const auto number = [file](const char* key, std::uint64_t value) {
        CsvRow row(file); row.text(key); row.number(value); row.end();
    };
    const auto real = [file](const char* key, long double value) {
        CsvRow row(file); row.text(key); row.real(static_cast<double>(value)); row.end();
    };
    const auto text = [file](const char* key, const char* value) {
        CsvRow row(file); row.text(key); row.text(value); row.end();
    };
    text("execution_mode", config.output_enabled ? "CSP_PRELOAD_CST_RAW_LOADING" : "READ_ONLY_DRY_RUN");
    text("source_feedback_freshness", "UNKNOWN");
    text("source_feedback_timeout", "UNSUPPORTED");
    text("terminal_heartbeat", "ENFORCED_DURING_OUTPUT");
    text("driver_process_watchdog", "CST_TARGET_SEQUENCE_100MS");
    text("command_timeout", "UNSUPPORTED");
    number("active_joint_mask", config.state.active_mask);
    number("period_ns", kPeriodNs);
    number("log_ring_capacity", kLogCapacity);
    number("requested_fifo_priority", static_cast<std::uint64_t>(config.fifo_priority));
    { CsvRow row(file); row.text("requested_cpu"); row.signed_number(config.cpu); row.end(); }
    number("memory_lock_requested", config.lock_memory);
    number("max_consecutive_failed_reads_allowed", config.state.max_consecutive_failed_reads);
    number("snapshot_timeout_ns", config.state.snapshot_timeout_ns);
    real("max_arm_angular_speed_rad_s", config.state.max_arm_angular_speed);
    real("max_arm_linear_speed_m_s", config.state.max_arm_linear_speed);
    number("cycles", summary.cycles);
    number("valid_cycles", summary.valid_cycles);
    number("deadline_misses", summary.deadline_misses);
    number("skipped_periods", summary.skipped_periods);
    number("max_wakeup_jitter_ns", summary.max_wakeup_jitter_ns);
    number("max_cycle_execution_ns", summary.max_cycle_execution_ns);
    number("dt_samples", summary.dt_samples);
    number("min_actual_dt_ns", summary.dt_samples == 0 ? 0 : summary.min_actual_dt_ns);
    number("max_actual_dt_ns", summary.max_actual_dt_ns);
    real("mean_actual_dt_ns", summary.dt_samples == 0 ? 0 : summary.sum_actual_dt_ns / summary.dt_samples);
    real("mean_wakeup_jitter_ns", summary.cycles == 0 ? 0 : summary.sum_wakeup_jitter_ns / summary.cycles);
    real("mean_cycle_execution_ns", summary.cycles == 0 ? 0 : summary.sum_cycle_execution_ns / summary.cycles);
    number("total_reads", summary.reads.total_reads);
    number("successful_reads", summary.reads.successful_reads);
    number("failed_reads", summary.reads.failed_reads);
    number("consecutive_failed_reads", summary.reads.consecutive_failed_reads);
    number("semaphore_contention_count", summary.reads.semaphore_contention_count);
    number("maximum_read_duration_ns", summary.reads.maximum_read_duration_ns);
    number("dropped_record_count", dropped);
    number("records_formatted", formatted);
    number("logger_io_error", static_cast<std::uint64_t>(io_error));
    text("final_state", kStateNames[static_cast<std::size_t>(summary.final_state)]);
    text("final_fault", kFaultNames[static_cast<std::size_t>(summary.final_fault)]);
}
}  // namespace

DataLogger::DataLogger()
    : records_(std::make_unique<SpscRing<CycleRecord, kLogCapacity>>()),
      file_buffer_(std::make_unique<char[]>(kFileBufferBytes)) {}

DataLogger::~DataLogger() {
    if (thread_.joinable()) {
        finish(summary_);
    }
}

void DataLogger::set_io_error(int error) noexcept {
    int expected = 0;
    io_error_.compare_exchange_strong(expected, error == 0 ? EIO : error, std::memory_order_relaxed);
}

bool DataLogger::start(const std::string& path, const LoopConfig& config, std::string& error) {
    if(thread_.joinable()){error="recording already active";return false;}
    config_ = config;startup_done_=false;finish_requested_.store(false);io_error_.store(0);dropped_.store(0);
    records_=std::make_unique<SpscRing<CycleRecord,kLogCapacity>>();
    thread_ = std::thread(&DataLogger::worker, this, path);
    std::unique_lock<std::mutex> lock(startup_mutex_);
    startup_cv_.wait(lock, [this] { return startup_done_; });
    if (io_error() != 0) {
        error = std::string("CSV logger initialization failed: ") + std::strerror(io_error());
        finish(LoopSummary{});return false;
    }
    active_.store(true);return true;
}

CycleRecord* DataLogger::reserve_record() noexcept {
    if(!active_.load())return nullptr;
    producing_.store(true);
    if(!active_.load()){producing_.store(false);return nullptr;}
    auto* record = records_->reserve();
    if (record == nullptr) {
        dropped_.fetch_add(1, std::memory_order_relaxed);producing_.store(false);
    }
    return record;
}

void DataLogger::finish(const LoopSummary& summary) {
    active_.store(false);
    while(producing_.load())std::this_thread::yield();
    if (thread_.joinable()) {
        summary_ = summary;
        finish_requested_.store(true, std::memory_order_release);
        thread_.join();
    }
}

void DataLogger::worker(std::string path) {
    // Both files are opened exclusively; existing logs are never truncated.
    std::FILE* file = std::fopen(path.c_str(), "wx");
    std::FILE* summary_file = nullptr;
    if (file == nullptr) {
        set_io_error(errno);
    } else {
        const std::string summary_path = path + ".summary.csv";
        summary_file = std::fopen(summary_path.c_str(), "wx");
        if (summary_file == nullptr) {
            set_io_error(errno);
        } else if (std::setvbuf(file, file_buffer_.get(), _IOFBF, kFileBufferBytes) != 0) {
            set_io_error(EIO);
        } else {
            write_header(file);
            if (std::fflush(file) != 0) { set_io_error(errno); }
        }
    }
    {
        std::lock_guard<std::mutex> lock(startup_mutex_);
        startup_done_ = true;
    }
    startup_cv_.notify_one();
    if (io_error() != 0) {
        if (file != nullptr) { std::fclose(file); }
        if (summary_file != nullptr) { std::fclose(summary_file); }
        return;
    }

    std::uint64_t formatted = 0,first_time=0;
    LoopSummary recorded{};
    CycleRecord record{};
    for (;;) {
        const bool finishing = finish_requested_.load(std::memory_order_acquire);
        if (records_->try_pop(record)) {
            if (io_error() == 0) {
                if(!first_time)first_time=record.actual_wakeup_ns;
                write_record(file, record,first_time);
                ++recorded.cycles;recorded.valid_cycles+=record.cycle_valid;
                recorded.deadline_misses+=record.deadline_miss;
                recorded.skipped_periods+=record.skipped_after_cycle;
                if(record.actual_dt_valid){
                    ++recorded.dt_samples;recorded.sum_actual_dt_ns+=record.actual_dt_ns;
                    recorded.min_actual_dt_ns=std::min(recorded.min_actual_dt_ns,record.actual_dt_ns);
                    recorded.max_actual_dt_ns=std::max(recorded.max_actual_dt_ns,record.actual_dt_ns);
                }
                recorded.sum_wakeup_jitter_ns+=record.wakeup_jitter_ns;
                if(record.wakeup_jitter_ns>0)recorded.max_wakeup_jitter_ns=std::max(recorded.max_wakeup_jitter_ns,static_cast<std::uint64_t>(record.wakeup_jitter_ns));
                recorded.sum_cycle_execution_ns+=record.cycle_execution_ns;
                recorded.max_cycle_execution_ns=std::max(recorded.max_cycle_execution_ns,record.cycle_execution_ns);
                ++recorded.reads.total_reads;
                recorded.reads.successful_reads+=record.feedback.snapshot_read_ok;
                recorded.reads.failed_reads+=!record.feedback.snapshot_read_ok;
                recorded.reads.consecutive_failed_reads=record.reads.consecutive_failed_reads;
                recorded.reads.semaphore_contention_count+=record.feedback.read_status==ReadStatus::Contended;
                recorded.reads.maximum_read_duration_ns=std::max(recorded.reads.maximum_read_duration_ns,record.feedback.snapshot_read_duration_ns);
                if (std::ferror(file) != 0) {
                    set_io_error(errno);
                } else {
                    ++formatted;
                    if (formatted % 1000 == 0 && std::fflush(file) != 0) {
                        set_io_error(errno);
                    }
                }
            }
        } else if (finishing) {
            break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    if (std::fclose(file) != 0) { set_io_error(errno); }
    recorded.final_state=summary_.final_state;recorded.final_fault=summary_.final_fault;
    write_summary(summary_file, recorded, config_, dropped_records(), formatted, io_error());
    if (std::fclose(summary_file) != 0) { set_io_error(errno); }
}

}  // namespace jc
