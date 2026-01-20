#include <torch/torch.h>
#include <opencv2/opencv.hpp>

#include "include/FeatureMatcher.h"

using FeatDict = c10::Dict<std::string, at::Tensor>;

static at::Tensor kps_to_tensor_N2(const std::vector<cv::KeyPoint>& kps, torch::Device device) {
    const int64_t N = (int64_t)kps.size();
    at::Tensor t = torch::empty({N, 2}, torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));
    auto a = t.accessor<float, 2>();
    for (int64_t i = 0; i < N; ++i) {
        a[i][0] = kps[(size_t)i].pt.x;
        a[i][1] = kps[(size_t)i].pt.y;
    }
    return t.to(device);
}

static at::Tensor desc_to_tensor_ND(const cv::Mat& desc, torch::Device device) {
    CV_Assert(desc.type() == CV_32F);
    cv::Mat d = desc.isContinuous() ? desc : desc.clone();
    const int64_t N = d.rows;
    const int64_t D = d.cols;
    // from_blob is a view; clone() makes it independent of cv::Mat memory
    at::Tensor t = torch::from_blob(
        (void*)d.ptr<float>(),
        {N, D},
        torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU)
    ).clone();
    return t.to(device);
}

static FeatDict make_feats_like_aliked(
    const std::vector<cv::KeyPoint>& kps,
    const cv::Mat& desc,
    int width, int height,
    torch::Device device
) {
    FeatDict f;
    f.insert("keypoints", kps_to_tensor_N2(kps, device));          
    f.insert("descriptors", desc_to_tensor_ND(desc, device));      
    f.insert("image_size",
            torch::tensor({(float)width, (float)height}, torch::TensorOptions().dtype(torch::kFloat32))
                .unsqueeze(0).to(device));                       
    return f;
}

static std::vector<cv::DMatch> lightglue_to_dmatches(
        const at::Tensor& matches0,                 
        const at::Tensor* matching_scores0 = nullptr, 
        float min_score = 0.0f
    ) {
        // Move to CPU + contiguous
        at::Tensor m = matches0.to(at::kCPU).contiguous();
        if (m.dim() == 2) m = m.squeeze(0); 
        TORCH_CHECK(m.dim() == 1, "matches0 must be [N0] or [1,N0]");
        m = m.to(at::kLong);                

        at::Tensor s;
        bool has_scores = (matching_scores0 != nullptr);
        if (has_scores) {
            s = matching_scores0->to(at::kCPU).contiguous();
            if (s.dim() == 2) s = s.squeeze(0);    
            TORCH_CHECK(s.dim() == 1, "matching_scores0 must be [N0] or [1,N0]");
            s = s.to(at::kFloat);                  
            TORCH_CHECK(s.size(0) == m.size(0), "scores length must match matches0 length");
        }

        auto m_acc = m.accessor<int64_t, 1>();
        const int64_t N0 = m.size(0);

        std::vector<cv::DMatch> out;
        out.reserve((size_t)N0);

        if (has_scores) {
            auto s_acc = s.accessor<float, 1>();
            for (int64_t i = 0; i < N0; ++i) {
                int64_t j = m_acc[i];
                if (j < 0) continue;

                float score = s_acc[i];
                if (score < min_score) continue;

                cv::DMatch dm;
                dm.queryIdx = (int)i;       
                dm.trainIdx = (int)j;        
                dm.imgIdx   = 0;
                dm.distance = 1.0f - score;  
                out.push_back(dm);
            }
        } else {
            for (int64_t i = 0; i < N0; ++i) {
                int64_t j = m_acc[i];
                if (j < 0) continue;

                cv::DMatch dm;
                dm.queryIdx = (int)i;
                dm.trainIdx = (int)j;
                dm.imgIdx   = 0;
                dm.distance = 0.0f;
                out.push_back(dm);
            }
        }

        return out;
    }

    std::vector<cv::DMatch> ANYFEATURE_VSLAM::FeatureMatcher::lightglueMatching(
        const std::vector<cv::KeyPoint>& kps1, const cv::Mat& desc1,
        const std::vector<cv::KeyPoint>& kps2, const cv::Mat& desc2,
        float min_score
    ){

        FeatDict f0 = make_feats_like_aliked(kps1, desc1, imageWidth, imageHeight, *torch_device);
        FeatDict f1 = make_feats_like_aliked(kps2, desc2, imageWidth, imageHeight, *torch_device);

        auto matches01 = matcher_lightglue->forward(f0, f1);
        const auto& matches0 = matches01.at("matches0");
        const auto& scores0  = matches01.at("matching_scores0");
        std::vector<cv::DMatch> matches = lightglue_to_dmatches(matches0, &scores0, min_score);    
        return matches;
    }