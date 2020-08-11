/*
 *  BasicDrawableInstanceBuilderGLES.cpp
 *  WhirlyGlobeLib
 *
 *  Created by Steve Gifford on 5/10/19.
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

#import "BasicDrawableInstanceBuilderGLES.h"

namespace WhirlyKit
{

BasicDrawableInstanceBuilderGLES::BasicDrawableInstanceBuilderGLES(const std::string &name,Scene *scene)
: BasicDrawableInstanceBuilder(name,scene), drawableGotten(false)
{
    BasicDrawableInstanceGLES *drawInstGL = new BasicDrawableInstanceGLES(name);
    drawInst = drawInstGL;
    
    Init();
    drawInstGL->instBuffer = 0;
    drawInstGL->vertArrayObj = 0;
}
    
BasicDrawableInstanceBuilderGLES::~BasicDrawableInstanceBuilderGLES()
{
    if (!drawableGotten && drawInst)
        delete drawInst;
}

BasicDrawableInstance *BasicDrawableInstanceBuilderGLES::getDrawable()
{
    drawableGotten = true;
    return drawInst;
}

}
