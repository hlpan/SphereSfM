/************************************************************************/
// project : colmap

// author : jiangsan, Shenzhen University.

// e-mail: jiangsan@szu.edu.cn

// date : 2023-5-19

/************************************************************************/

#include "exe/sphere.h"

#include "base/reconstruction.h"
#include "base/sphere_camera.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace colmap {

int RunSphereCubicReprojecter(int argc, char** argv) {
  std::string input_path;
  std::string output_path;
  std::string image_ids = "0,1,2,3,4,5";
  int image_size = 0;
  double field_of_view = 45.0;

  OptionManager options;
  options.AddImageOptions();
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("image_ids", &image_ids);
  options.AddDefaultOption("image_size", &image_size);
  options.AddDefaultOption("field_of_view", &field_of_view);
  options.Parse(argc, argv);

  CreateDirIfNotExists(output_path);

  Reconstruction reconstruction;
  reconstruction.Read(input_path);
  reconstruction.ExportPerspectiveCubic(output_path, *options.image_path,
                                        CSVToVector<int>(image_ids), image_size,
                                        field_of_view);

  return EXIT_SUCCESS;
}

int RunSphereStereoExporter(int argc, char** argv) {
  std::string input_path;
  std::string output_path;

  // 逗号分隔：可一次跑多组基线间隔（例如 "3,10,30"）
  std::string baseline_intervals = "10";

  // 环角（单位度），在“垂直于运动方向的环”上的等分角度
  std::string ring_degrees = "0,60,120,180,240,300";

  // 目标针孔相机规格（与 ExportPerspectiveCubic 的 image_size/field_of_view 类似）
  int image_size = 0;                 // <=0 则按 ERP 高度/2
  double field_of_view = 75.0;        // 目标针孔水平 FOV（度）

  // 世界“上”向量（有 IMU 用重力方向；否则用 0,1,0）
  std::string world_up = "0,1,0";

  // 过短基线剔除阈值（米）
  double min_baseline_m = 0.05;

  // ERP mask 目录（可选），为空则不导出 mask
  std::string mask_path = "";
  OptionManager options;
  options.AddImageOptions();  // 提供 --image_path
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("baseline_intervals", &baseline_intervals);
  options.AddDefaultOption("ring_degrees", &ring_degrees);
  options.AddDefaultOption("image_size", &image_size);
  options.AddDefaultOption("field_of_view", &field_of_view);
  options.AddDefaultOption("world_up", &world_up);
  options.AddDefaultOption("min_baseline_m", &min_baseline_m);
  options.AddDefaultOption("mask_path", &mask_path);
  options.Parse(argc, argv);

  CreateDirIfNotExists(output_path);

  // 读取重建
  Reconstruction reconstruction;
  reconstruction.Read(input_path);

  // 解析参数
  const auto baselines = CSVToVector<int>(baseline_intervals);
  const auto rings     = CSVToVector<double>(ring_degrees);
  const auto upv       = CSVToVector<double>(world_up);

  Eigen::Vector3d up(0, 1, 0);
  if (upv.size() == 3) up = Eigen::Vector3d(upv[0], upv[1], upv[2]).normalized();

  // 依次导出不同基线长度的双目对
  for (const int k : baselines) {
    if (k <= 0) {
      std::cout << "Skip invalid baseline_interval=" << k << std::endl;
      continue;
    }
    reconstruction.ExportStereoPairs(
        output_path,                  // out_root
        *options.image_path,          // ERP 原图目录
        image_size,                   // 针孔输出尺寸
        field_of_view,                // 针孔水平FOV（度）
        k,                            // baseline_interval（帧间隔）
        rings,                        // 环角集合（度）
        up,                           // 世界上向量
        min_baseline_m,                // 最小基线（米）
        mask_path
    );
  }

  return EXIT_SUCCESS;
}

}  // namespace colmap
