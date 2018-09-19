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
vtkStandardNewMacro(vtkSlicerVideoCamerasAutomaticSegmentationResult);

//----------------------------------------------------------------------------
vtkSlicerVideoCamerasLogic::vtkSlicerVideoCamerasLogic()
  : Threader(vtkSmartPointer<vtkMultiThreader>::New())
  , ThreadID(-1)
  , ThreadMutexLock(vtkSmartPointer<vtkMutexLock>::New())
  , EventQueueMutex(vtkSmartPointer<vtkMutexLock>::New())
{
}

//----------------------------------------------------------------------------
vtkSlicerVideoCamerasLogic::~vtkSlicerVideoCamerasLogic()
{

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
void vtkSlicerVideoCamerasLogic::SegmentCircleInImageAsync(vtkMRMLScalarVolumeNode* volumeNode,
    vtkMRMLVideoCameraNode* cameraNode,
    vtkMRMLLinearTransformNode* tipToCameraNode,
    double colorRange1Low[3],
    double colorRange1High[3],
    double colorRange2Low[3],
    double colorRange2High[3],
    double minDist,
    double param1,
    double param2,
    int minRadius,
    int maxRadius)
{
  this->ThreadMutexLock->Lock();

  vtkAutoSegmentationParameters* segmentationParameters = vtkAutoSegmentationParameters::New();

  segmentationParameters->SetAutomaticSegmentationImageNode(volumeNode);
  segmentationParameters->SetAutomaticSegmentationCameraNode(cameraNode);
  segmentationParameters->ColorRanges[0][0] = colorRange1Low[0];
  segmentationParameters->ColorRanges[0][1] = colorRange1Low[1];
  segmentationParameters->ColorRanges[0][2] = colorRange1Low[2];
  segmentationParameters->ColorRanges[1][0] = colorRange1High[0];
  segmentationParameters->ColorRanges[1][1] = colorRange1High[1];
  segmentationParameters->ColorRanges[1][2] = colorRange1High[2];
  segmentationParameters->ColorRanges[2][0] = colorRange2Low[0];
  segmentationParameters->ColorRanges[2][1] = colorRange2Low[1];
  segmentationParameters->ColorRanges[2][2] = colorRange2Low[2];
  segmentationParameters->ColorRanges[3][0] = colorRange2High[0];
  segmentationParameters->ColorRanges[3][1] = colorRange2High[1];
  segmentationParameters->ColorRanges[3][2] = colorRange2High[2];
  segmentationParameters->MinDist = minDist;
  segmentationParameters->Param1 = param1;
  segmentationParameters->Param2 = param2;
  segmentationParameters->MinRadius = minRadius;
  segmentationParameters->MaxRadius = maxRadius;

  segmentationParameters->ParentLogic = this;

  if (volumeNode != nullptr)
  {
    this->ThreadID = this->Threader->SpawnThread((vtkThreadFunctionType)&vtkSlicerVideoCamerasLogic::SegmentImageThreadFunction, segmentationParameters);
  }

  this->ThreadMutexLock->Unlock();
}

//----------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::PeriodicSegmentationProcess()
{
  this->EventQueueMutex->Lock();

  if (this->ResultQueue.size() == 0)
  {
    return;
  }
  vtkSlicerVideoCamerasAutomaticSegmentationResult* result = this->ResultQueue.front();
  this->InvokeEvent(VideoCameraLogicEventType::AutomaticSegmentationResultEvent, (void*)result);
  this->ResultQueue.pop();

  this->EventQueueMutex->Unlock();
}

//----------------------------------------------------------------------------
void* vtkSlicerVideoCamerasLogic::SegmentImageThreadFunction(void* ptr)
{
  vtkMultiThreader::ThreadInfo* vinfo = static_cast<vtkMultiThreader::ThreadInfo*>(ptr);
  vtkAutoSegmentationParameters* params = static_cast<vtkAutoSegmentationParameters*>(vinfo->UserData);

  auto lastModifiedTime = 0;
  auto modifiedTime = 0;

  modifiedTime = params->AutomaticSegmentationImageNode->GetMTime();
  if (modifiedTime == lastModifiedTime)
  {
    params->Delete();
    return 0;
  }
  lastModifiedTime = modifiedTime;
  auto imageData = params->AutomaticSegmentationImageNode->GetImageData();
  auto dims = imageData->GetDimensions();

  auto intrinsics = params->AutomaticSegmentationCameraNode->GetIntrinsicMatrix();
  cv::Mat cvIntrinsics(3, 3, CV_64F, intrinsics->GetData());
  auto distortion = params->AutomaticSegmentationCameraNode->GetDistortionCoefficients();
  assert(distortion->GetNumberOfValues() == 5 || distortion->GetNumberOfValues() == 8 || distortion->GetNumberOfValues() == 12 || distortion->GetNumberOfValues() == 14);
  cv::Mat cvDistortion(1, distortion->GetNumberOfValues(), CV_64F);
  for (int i = 0; i < distortion->GetNumberOfValues(); ++i)
  {
    cvDistortion.at<double>(0, i) = distortion->GetValue(i);
  }

  // Undistort the image, copy image
  cv::Mat cvImage;
  cv::Mat(dims[1], dims[0], CV_8UC3, imageData->GetScalarPointer(0, 0, 0)).copyTo(cvImage);

  auto cvUndistorted = cv::Mat(dims[1], dims[0], CV_8UC3);
  cv::undistort(cvImage, cvUndistorted, cvIntrinsics, cvDistortion);
  cv::flip(cvUndistorted, cvUndistorted, 0);

  // Convert RGB image to HSV image
  cv::Mat cvHSV;
  cv::cvtColor(cvUndistorted, cvHSV, cv::COLOR_RGB2HSV);

  // Filter everything except desired ranges
  cv::Mat cvThresholdLower, cvThresholdUpper;
  cv::inRange(cvHSV,
              cv::Scalar(params->ColorRanges[0][0],
                         params->ColorRanges[0][1],
                         params->ColorRanges[0][2]),
              cv::Scalar(params->ColorRanges[1][0],
                         params->ColorRanges[1][1],
                         params->ColorRanges[1][2]),
              cvThresholdLower);
  cv::inRange(cvHSV,
              cv::Scalar(params->ColorRanges[2][0],
                         params->ColorRanges[2][1],
                         params->ColorRanges[2][2]),
              cv::Scalar(params->ColorRanges[3][0],
                         params->ColorRanges[3][1],
                         params->ColorRanges[3][2]),
              cvThresholdUpper);
  cv::Mat cvMask;
  cv::addWeighted(cvThresholdLower, 1.0, cvThresholdUpper, 1.0, 0.0, cvMask);

  // Create a gaussian & median blur filter
  cv::medianBlur(cvMask, cvMask, 5);
  cv::GaussianBlur(cvMask, cvMask, cv::Size(9, 9), 2, 2);

  // Apply the Hough Transform to find the circles
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(cvMask, circles,
                   cv::HOUGH_GRADIENT,
                   2,
                   cvMask.rows / params->MinDist,
                   params->Param1,
                   params->Param2,
                   params->MinRadius,
                   params->MaxRadius);

  // Extract most likely center
  cv::Point2f center;
  float radius;
  if (circles.size() > 0)
  {
    center = cv::Point2f(circles[0][0], circles[0][1]);
    radius = circles[0][2];
  }
  else if (circles.size() == 0)
  {
    cv::Mat cvCannyOutput;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    int thresh = 100;
    int max_thresh = 255;
    cv::RNG rng(12345);

    cv::medianBlur(cvMask, cvMask, 3);

    // Detect edges using canny
    cv::Canny(cvMask, cvCannyOutput, thresh, thresh * 2, 3);

    // Find contours
    cv::findContours(cvCannyOutput, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f> centers(contours.size());
    std::vector<float> radii(contours.size());

    for (std::vector<std::vector<cv::Point>>::size_type i = 0; i < contours.size(); i++)
    {
      cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true); // Finds polygon
      boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));      // Finds rectangle
      cv::minEnclosingCircle((cv::Mat)contours_poly[i], centers[i], radii[i]); // Finds circle
    }

    if (centers.size() == 0)
    {
      params->Delete();
      return 0;
    }
    else
    {
      center = centers[0];
      radius = radii[0];
    }
  }

  vtkSlicerVideoCamerasAutomaticSegmentationResult* result = vtkSlicerVideoCamerasAutomaticSegmentationResult::New();
  result->CenterX = center.x;
  result->CenterY = center.y;
  result->Radius = radius;
  vtkNew<vtkMatrix4x4> temp;
  params->TipToCameraNode->GetMatrixTransformToParent(temp);
  result->Tip_Camera[0] = temp->GetElement(0, 3);
  result->Tip_Camera[1] = temp->GetElement(1, 3);
  result->Tip_Camera[2] = temp->GetElement(2, 3);
  params->ParentLogic->QueueSegmentationResult(result);

  // Signal to the threader that this thread has become free
  vinfo->ActiveFlagLock->Lock();
  (*vinfo->ActiveFlag) = 0;
  vinfo->ActiveFlagLock->Unlock();

  params->Delete();
  return 0;
}

//----------------------------------------------------------------------------
void vtkSlicerVideoCamerasLogic::QueueSegmentationResult(vtkSlicerVideoCamerasAutomaticSegmentationResult* result)
{
  this->EventQueueMutex->Lock();
  this->ResultQueue.push(result);
  this->EventQueueMutex->Unlock();
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
