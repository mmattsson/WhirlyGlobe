/*
 *  WGScreenLabel.h
 *  WhirlyGlobeComponent
 *
 *  Created by Steve Gifford on 7/24/12.
 *  Copyright 2011-2012 mousebird consulting
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

#import "MaplyScreenLabel.h"

@implementation MaplyScreenLabel

@synthesize userObject;
@synthesize loc;
@synthesize rotation;
@synthesize size;
@synthesize text;
@synthesize iconImage;
@synthesize iconSize;
@synthesize offset;
@synthesize color;
@synthesize selectable;
@synthesize layoutImportance;
@synthesize layoutPlacement;

- (id)init
{
    self = [super init];
    if (!self)
        return nil;
    
    selectable = true;
    layoutImportance = MAXFLOAT;
    layoutPlacement = kMaplyLayoutRight | kMaplyLayoutLeft | kMaplyLayoutAbove | kMaplyLayoutBelow;
    
    return self;
}


@end
