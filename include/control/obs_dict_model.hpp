#pragma once

#include "tensorrt/InferenceEngine.h"

#include <cstddef>
#include <string>

namespace kist {

// TRT wrapper for the encoder/decoder models, which share one shape:
// a single float input tensor "obs_dict" and a single float output tensor.
// (gear_sonic has separate EncoderEngine / PolicyEngine classes with
// identical plumbing; one class serves both here.)
//
//   encoder: obs_dict[1,1751] -> encoded_tokens[1,64]  (SONIC v1.1)
//   decoder: obs_dict[1, 994] -> action[1,29]
class ObsDictModel {
public:
    // Converts the ONNX to a cached TRT engine and validates that tensor
    // names/dims match. expected_* of 0 skips that dimension check.
    bool Initialize(const std::string& onnx_path,
                    const std::string& output_name,
                    size_t expected_input_dim,
                    size_t expected_output_dim,
                    Precision precision = Precision::FP32);

    // Caller fills input() [input_dim floats], then Infer(), then reads
    // output() [output_dim floats]. Synchronous on a dedicated stream.
    bool Infer();

    float*       input()  { return input_buf_.data(); }
    const float* output() const { return output_buf_.data(); }
    size_t       input_dim()  const { return input_buf_.size(); }
    size_t       output_dim() const { return output_buf_.size(); }

    ~ObsDictModel();

private:
    TRTInferenceEngine   engine_;
    TPinnedVector<float> input_buf_;
    TPinnedVector<float> output_buf_;
    std::string          output_name_;
    cudaStream_t         stream_{nullptr};
};

} // namespace kist
