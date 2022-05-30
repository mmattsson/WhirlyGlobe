/*  ScreenSpaceDrawable.h
 *  WhirlyGlobeLib
 *
 *  Created by Steve Gifford on 8/24/14.
 *  Copyright 2011-2022 mousebird consulting.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#import "BasicDrawableBuilder.h"
#import "SceneRenderer.h"
#import "BaseInfo.h"

namespace WhirlyKit
{
    
// Modifies the uniform values of a given shader right before the
//  screenspace's Basic Drawables are rendered
class ScreenSpaceTweaker : public BasicDrawableTweaker
{
public:
    virtual void tweakForFrame(Drawable *inDraw,RendererFrameInfo *frameInfo) = 0;
    
    TimeInterval startTime;
    bool keepUpright;
    bool activeRot;
    bool motion;
};

/// Wrapper for building screen space drawables
struct ScreenSpaceDrawableBuilder : virtual public BasicDrawableBuilder
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // Construct with or without motion support
    ScreenSpaceDrawableBuilder() = default;
    virtual void ScreenSpaceInit(bool hasMotion,bool hasRotation,bool buildAnyway/* = false*/);

    // If we've got a rotation, we set this to keep the image facing upright
    //  probably because it's text.
    void setKeepUpright(bool in) { keepUpright = in; }
    // Time we start counting from for motion
    void setStartTime(TimeInterval inStartTime) { startTime = inStartTime; }
    // Time we start counting from for motion
    TimeInterval getStartTime() const { return startTime; }

    // Each vertex has an offset on the screen
    void addOffset(const Point2f &offset);
    void addOffset(const Point2d &offset);

    // Add a direction to the attribute list (for animation)
    void addDir(const Point3f &dir);
    void addDir(const Point3d &dir);
    
    // Add a rotation vector to the attribute list
    void addRot(const Point3f &dir);
    void addRot(const Point3d &dir) { addRot(Point3f(dir.cast<float>())); }
    
    // Apply a scale expression
    void setScaleExpression(FloatExpressionInfoRef exp) { scaleExp = std::move(exp); }

    virtual void setupTweaker(const DrawableTweakerRef &inTweaker) const override;

protected:
    // Call ScreenSpaceInit instead
    virtual void Init() override { BasicDrawableBuilder::Init(); }

    bool motion = false;
    bool rotation = false;
    bool keepUpright = false;
    int offsetIndex = -1;
    int dirIndex = -1;
    int rotIndex = -1;
    TimeInterval startTime = 0.0;
    FloatExpressionInfoRef scaleExp;
    FloatExpressionInfoRef opacityExp;
    ColorExpressionInfoRef colorExp;
};
    
}
