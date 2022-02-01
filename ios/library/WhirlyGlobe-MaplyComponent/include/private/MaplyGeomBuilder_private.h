/*
 *  MaplyGeomModelBuilder_private.h
 *  WhirlyGlobeComponent
 *
 *  Created by Steve Gifford on 1/20/16
 *  Copyright 2011-2022 mousebird consulting
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

#import <vector>
#import <set>
#import "helpers/MaplyGeomBuilder.h"
#import <WhirlyGlobe_iOS.h>
#import "MaplyRenderController_private.h"
#import "MaplyGeomModel_private.h"

@interface MaplyGeomBuilder()
{
@public
    __weak NSObject<MaplyRenderControllerProtocol> *viewC;

    std::vector<WhirlyKit::GeometryRaw> rawGeom;
    std::vector<WhirlyKit::GeomStringWrapper> strings;
    std::vector<id> textures;
}
@end
