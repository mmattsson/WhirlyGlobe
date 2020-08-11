/*
 *  ScreenSpaceDrawableBuilderMTL.mm
 *  WhirlyGlobeLib
 *
 *  Created by Steve Gifford on 5/16/19.
 *  Copyright 2011-2019 mousebird consulting
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
 *
 */

#import "ScreenSpaceDrawableBuilderMTL.h"
#import "DefaultShadersMTL.h"
#import "RawData_NSData.h"

namespace WhirlyKit
{
    
ScreenSpaceDrawableBuilderMTL::ScreenSpaceDrawableBuilderMTL(const std::string &name,Scene *scene)
    : BasicDrawableBuilderMTL(name,scene)
{
    this->scene = scene;
}

void ScreenSpaceDrawableBuilderMTL::Init(bool hasMotion,bool hasRotation,bool buildAnyway)
{
    // TODO: Fix this.  It's dumb.
    if (basicDraw)
        delete basicDraw;
    basicDraw = new BasicDrawableMTL("Screen Space");
    // Need the entries even if we don't bother to fill them in
    ScreenSpaceDrawableBuilder::Init(hasMotion,hasRotation,true);
    
    // Wire up the buffers
    // TODO: Merge these into a single data structure
    ((VertexAttributeMTL *)basicDraw->vertexAttributes[offsetIndex])->bufferIndex = WhirlyKitShader::WKSVertexScreenSpaceOffsetAttribute;
    ((VertexAttributeMTL *)basicDraw->vertexAttributes[rotIndex])->bufferIndex = WhirlyKitShader::WKSVertexScreenSpaceRotAttribute;
    ((VertexAttributeMTL *)basicDraw->vertexAttributes[dirIndex])->bufferIndex = WhirlyKitShader::WKSVertexScreenSpaceDirAttribute;
}

ScreenSpaceTweaker *ScreenSpaceDrawableBuilderMTL::makeTweaker()
{
    return NULL;
}

BasicDrawable *ScreenSpaceDrawableBuilderMTL::getDrawable()
{
    if (drawableGotten)
        return BasicDrawableBuilderMTL::getDrawable();
    
    BasicDrawable *theDraw = BasicDrawableBuilderMTL::getDrawable();
    
    WhirlyKitShader::UniformScreenSpace uniSS;
    bzero(&uniSS,sizeof(uniSS));
    uniSS.keepUpright = keepUpright;
    if (motion) {
        theDraw->motion = true;
        uniSS.startTime = startTime - scene->getBaseTime();
    } else
        uniSS.startTime = 0.0;
    uniSS.activeRot = rotation;
    uniSS.hasMotion = motion;
    
    BasicDrawable::UniformBlock uniBlock;
    uniBlock.blockData = RawDataRef(new RawNSDataReader([[NSData alloc] initWithBytes:&uniSS length:sizeof(uniSS)]));
    uniBlock.bufferID = WhirlyKitShader::WKSUniformScreenSpaceEntry;
    theDraw->setUniBlock(uniBlock);

    
    return theDraw;
}
    
}