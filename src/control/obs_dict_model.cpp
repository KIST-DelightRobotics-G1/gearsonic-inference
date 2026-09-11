#include "control/obs_dict_model.hpp"

#include <iostream>
#include <numeric>

namespace kist {

static size_t tensor_dim(const TRTInferenceEngine& engine, const std::string& name) {
    std::vector<int64_t> shape;
    if (!engine.GetTensorShape(name, shape))
        return 0;
    size_t n = 1;
    for (auto d : shape)
        if (d > 0) n *= static_cast<size_t>(d);
    return n;
}

bool ObsDictModel::Initialize(const std::string& onnx_path,
                              const std::string& output_name,
                              size_t expected_input_dim,
                              size_t expected_output_dim,
                              Precision precision) {
    output_name_ = output_name;

    Options opts;
    opts.precision = precision;
    opts.deviceID  = 0;

    std::string trt_path;
    if (!ConvertONNXToTRT(opts, onnx_path, trt_path)) {
        std::cerr << "[ObsDictModel] ConvertONNXToTRT failed: " << onnx_path << "\n";
        return false;
    }
    if (!engine_.Initialize(trt_path, 0) || !engine_.InitInputs()) {
        std::cerr << "[ObsDictModel] engine init failed: " << onnx_path << "\n";
        return false;
    }

    auto inputs  = engine_.GetInputTensorNames();
    auto outputs = engine_.GetOutputTensorNames();
    if (inputs.size() != 1 || inputs[0] != "obs_dict" ||
        outputs.size() != 1 || outputs[0] != output_name_) {
        std::cerr << "[ObsDictModel] unexpected tensor names in " << onnx_path << "\n";
        return false;
    }

    size_t in_dim  = tensor_dim(engine_, "obs_dict");
    size_t out_dim = tensor_dim(engine_, output_name_);
    if ((expected_input_dim  && in_dim  != expected_input_dim) ||
        (expected_output_dim && out_dim != expected_output_dim)) {
        std::cerr << "[ObsDictModel] " << onnx_path << " dims " << in_dim << "->" << out_dim
                  << ", expected " << expected_input_dim << "->" << expected_output_dim << "\n";
        return false;
    }

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        std::cerr << "[ObsDictModel] cudaStreamCreate failed\n";
        return false;
    }
    input_buf_.resize(in_dim);
    output_buf_.resize(out_dim);
    return true;
}

bool ObsDictModel::Infer() {
    engine_.SetInputDataAsync("obs_dict", input_buf_, stream_);
    if (!engine_.Enqueue(stream_)) {
        std::cerr << "[ObsDictModel] Enqueue failed\n";
        return false;
    }
    engine_.GetOutputDataAsync(output_name_, output_buf_, stream_);
    cudaStreamSynchronize(stream_);
    return true;
}

void ObsDictModel::release() {
    engine_.Destroy();
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

ObsDictModel::~ObsDictModel() {
    release();
}

} // namespace kist
