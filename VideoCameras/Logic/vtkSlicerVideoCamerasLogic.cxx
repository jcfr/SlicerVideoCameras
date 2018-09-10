/*=auto=========================================================================

Portions (c) Copyright 2018 Robarts Research Institute. All Rights Reserved.

See COPYRIGHT.txt
or http://www.slicer.org/copyright/copyright.txt for details.

Program:   3D Slicer
Module:    $RCSfile: vtkSlicerVideoCamerasLogic.cxx,v $
Date:      $Date: 2018/6/16 10:54:09 $
Version:   $Revision: 1.0 $

=========================================================================auto=*/

// VideoCameras Logic includes
#include "vtkSlicerVideoCamerasLogic.h"
#include "vtkMRMLVideoCameraStorageNode.h"

// MRML includes
#include <vtkMRMLScalarVolumeNode.h>
#include <vtkMRMLScene.h>

// VTK includes
#include <vtkIntArray.h>
#include <vtkMultiThreader.h>
#include <vtkMutexLock.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtksys/SystemTools.hxx>
#include <vtkImageData.h>

// STD includes
#include <cassert>
#include <chrono>
#include <thread>

// OpenCV includes
#include <opencv2/imgproc.hpp>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerVideoCamerasLogic);
vtkStandardNewMacro(vtkSlicerVideoCamerasLogic::vtkAutoSegmentationParameters);

//----------------------------------------------------------------------------
vtkSlicerVideoCamerasLogic::vtkSlicerVideoCamerasLogic()
  : SegmentationParameters(vtkAutoSegmentationParameters::New())
  , Threader(vtkSmartPointer<vtkMultiThreader>::New())
  , ThreadID(-1)
{
}

//----------------------------------------------------------------------------
vtkSlicerVideoCamerasLogic::~vtkSlicerVideoCamerasLogic()
{
  SegmentationParameters->Delete();
}

//----------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
vtkMRMLVideoCameraNode* vtkSlicerVideoCamerasLogic::AddVideoCamera(const char* filename, const char* nodeName /*= NULL*/)
{
  if (this->GetMRMLScene() == NULL || filename == NULL)
  {
    return NULL;
  }
  vtkNew<vtkMRMLVideoCameraNode> videoCameraNode;
  vtkNew<vtkMRMLVideoCameraStorageNode> mStorageNode;
  vtkSmartPointer<vtkMRMLStorageNode> storageNode;

  mStorageNode->SetFileName(filename);

  const std::string fname(filename);
  // the VideoCamera node name is based on the file name (itksys call should work even if file is not on disk yet)
  std::string name = vtksys::SystemTools::GetFilenameName(fname);

  // check to see which node can read this type of file
  if (mStorageNode->SupportedFileType(name.c_str()))
  {
    storageNode = mStorageNode.GetPointer();
  }

  if (storageNode != NULL)
  {
    std::string baseName = vtksys::SystemTools::GetFilenameWithoutExtension(fname);
    std::string uname(this->GetMRMLScene()->GetUniqueNameByString(baseName.c_str()));
    videoCameraNode->SetName(uname.c_str());

    this->GetMRMLScene()->SaveStateForUndo();

    this->GetMRMLScene()->AddNode(storageNode.GetPointer());

    // Set the scene so that SetAndObserve[Display|Storage]NodeID can find the
    // node in the scene (so that DisplayNodes return something not empty)
    videoCameraNode->SetScene(this->GetMRMLScene());
    videoCameraNode->SetAndObserveStorageNodeID(storageNode->GetID());

    this->GetMRMLScene()->AddNode(videoCameraNode.GetPointer());

    // now set up the reading
    vtkDebugMacro("AddVideoCamera: calling read on the storage node");
    int retval = storageNode->ReadData(videoCameraNode.GetPointer());
    if (retval != 1)
    {
      vtkErrorMacro("AddVideoCamera: error reading " << filename);
      this->GetMRMLScene()->RemoveNode(videoCameraNode.GetPointer());
      return NULL;
    }
  }
  else
  {
    vtkErrorMacro("Couldn't read file: " << filename);
    return NULL;
  }

  return videoCameraNode.GetPointer();
}

//----------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::StartAutomaticSegmentation(vtkMRMLScalarVolumeNode* volumeNode,
    vtkMRMLVideoCameraNode* cameraNode,
    const std::array<std::array<double, 3>, 4>& colorRanges,
    double minDist,
    double param1,
    double param2,
    int minRadius,
    int maxRadius)
{
  this->ThreadMutexLock->Lock();
  this->SegmentationParameters->SetAutomaticSegmentationImageNode(volumeNode);
  this->SegmentationParameters->SetAutomaticSegmentationCameraNode(cameraNode);
  this->SegmentationParameters->ColorRanges = colorRanges;
  this->SegmentationParameters->MinDist = minDist;
  this->SegmentationParameters->Param1 = param1;
  this->SegmentationParameters->Param2 = param2;
  this->SegmentationParameters->MinRadius = minRadius;
  this->SegmentationParameters->MaxRadius = maxRadius;

  if (volumeNode != nullptr)
  {
    this->ThreadRunFlag = true;
    this->ThreadID = this->Threader->SpawnThread((vtkThreadFunctionType)&vtkSlicerVideoCamerasLogic::SegmentImageThreadFunction, this);
  }

  this->ThreadMutexLock->Unlock();
}

//----------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::StopAutomaticSegmentation()
{
  this->ThreadMutexLock->Lock();

  this->SegmentationParameters->SetAutomaticSegmentationImageNode(nullptr);
  this->SegmentationParameters->SetAutomaticSegmentationCameraNode(nullptr);
  this->ThreadRunFlag = false;

  this->ThreadMutexLock->Unlock();
}

//----------------------------------------------------------------------------
void* vtkSlicerVideoCamerasLogic::SegmentImageThreadFunction(void* ptr)
{
  vtkMultiThreader::ThreadInfo* vinfo = static_cast<vtkMultiThreader::ThreadInfo*>(ptr);
  vtkSlicerVideoCamerasLogic* logic = static_cast<vtkSlicerVideoCamerasLogic*>(vinfo->UserData);

  auto lastModifiedTime = 0;
  auto modifiedTime = 0;
  while (logic->ThreadRunFlag)
  {
    modifiedTime = logic->SegmentationParameters->AutomaticSegmentationImageNode->GetMTime();
    if (modifiedTime == lastModifiedTime)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
      continue;
    }
    lastModifiedTime = modifiedTime;
    auto imageData = logic->SegmentationParameters->AutomaticSegmentationImageNode->GetImageData();
    auto dims = imageData->GetDimensions();

    auto intrinsics = logic->SegmentationParameters->AutomaticSegmentationCameraNode->GetIntrinsicMatrix();
    cv::Mat cvIntrinsics(3, 3, CV_64F, intrinsics->GetData());
    auto distortion = logic->SegmentationParameters->AutomaticSegmentationCameraNode->GetDistortionCoefficients();
    assert(distortion->GetNumberOfValues() == 5 || distortion->GetNumberOfValues() == 8 || distortion->GetNumberOfValues() == 12 || distortion->GetNumberOfValues() == 14);
    cv::Mat cvDistortion(1, distortion->GetNumberOfValues(), CV_64F);
    for (int i = 0; i < distortion->GetNumberOfValues(); ++i)
    {
      cvDistortion.at<double>(0, i) = distortion->GetValue(i);
    }

    // Undistort the image, no memory copy
    // TODO: should make copy in case others are using the same source image node
    auto cvImage = cv::Mat(dims[1], dims[0], CV_8UC3, imageData->GetScalarPointer(0, 0, 0));
    auto cvUndistorted = cv::Mat(dims[1], dims[0], CV_8UC3);
    cv::undistort(cvImage, cvUndistorted, cvIntrinsics, cvDistortion);
    cv::flip(cvUndistorted, cvUndistorted, 0);

    // Convert RGB image to HSV image
    cv::Mat hsv;
    cv::cvtColor(cvUndistorted, hsv, cv::COLOR_RGB2HSV);

    // Filter everything except desired ranges
    cv::Mat thresholdLower, thresholdUpper;
    cv::inRange(hsv,
                cv::Scalar(logic->SegmentationParameters->ColorRanges[0][0],
                           logic->SegmentationParameters->ColorRanges[0][1],
                           logic->SegmentationParameters->ColorRanges[0][2]),
                cv::Scalar(logic->SegmentationParameters->ColorRanges[1][0],
                           logic->SegmentationParameters->ColorRanges[1][1],
                           logic->SegmentationParameters->ColorRanges[1][2]),
                thresholdLower);
    cv::inRange(hsv,
                cv::Scalar(logic->SegmentationParameters->ColorRanges[2][0],
                           logic->SegmentationParameters->ColorRanges[2][1],
                           logic->SegmentationParameters->ColorRanges[2][2]),
                cv::Scalar(logic->SegmentationParameters->ColorRanges[3][0],
                           logic->SegmentationParameters->ColorRanges[3][1],
                           logic->SegmentationParameters->ColorRanges[3][2]),
                thresholdUpper);
    cv::Mat mask;
    cv::addWeighted(thresholdLower, 1.0, thresholdUpper, 1.0, 0.0, mask);
    // Create a Gaussian & median Blur Filter
    cv::medianBlur(mask, mask, 5);
    cv::GaussianBlur(mask, mask, cv::Size(9, 9), 2, 2);
    std::vector<cv::Vec3f> circles;
    cv::Mat cannyOutput;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // Apply the Hough Transform to find the circles
    cv::HoughCircles(mask, circles,
                     cv::HOUGH_GRADIENT,
                     2,
                     mask.rows / logic->SegmentationParameters->MinDist,
                     logic->SegmentationParameters->Param1,
                     logic->SegmentationParameters->Param2,
                     logic->SegmentationParameters->MinRadius,
                     logic->SegmentationParameters->MaxRadius);
    if (circles.size() > 0)
    {
      cv::Point2f center(circles[0][0], circles[0][1]);
      int radius = circles[0][2];
    }
    else if (circles.size() == 0)
    {
      int thresh = 100;
      int max_thresh = 255;
      cv::RNG rng(12345);

      cv::medianBlur(mask, mask, 3);

      // Detect edges using canny
      cv::Canny(mask, cannyOutput, thresh, thresh * 2, 3);

      // Find contours
      cv::findContours(cannyOutput, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point>> contours_poly(contours.size());
      std::vector<cv::Rect> boundRect(contours.size());
      std::vector<cv::Point2f> center(contours.size());
      std::vector<float> radius(contours.size());

      for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contours.size(); i++)
      {
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true); // Finds polygon
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));      // Finds rectangle
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]); // Finds circle
      }
    }

    // TODO : signal results some way
  }

  // Signal to the threader that this thread has become free
  vinfo->ActiveFlagLock->Lock();
  (*vinfo->ActiveFlag) = 0;
  vinfo->ActiveFlagLock->Unlock();
  return 0;
}

//---------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::SetMRMLSceneInternal(vtkMRMLScene* newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);

  vtkMRMLScene* scene = this->GetMRMLScene();

  // Nodes
  scene->RegisterNodeClass(vtkSmartPointer<vtkMRMLVideoCameraNode>::New());
  scene->RegisterNodeClass(vtkSmartPointer<vtkMRMLVideoCameraStorageNode>::New());
}

//---------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
}

//---------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}
