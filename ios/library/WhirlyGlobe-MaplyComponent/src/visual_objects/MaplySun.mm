/*  MaplySun.mm
 *  WhirlyGlobe-MaplyComponent
 *
 *  Created by Steve Gifford on 6/24/15.
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
 */

#import <WhirlyGlobe_iOS.h>
#import "visual_objects/MaplySun.h"
#import <AA+.h>

using namespace WhirlyKit;

@implementation MaplySun
{
    std::unique_ptr<Sun> sun;
}

- (_Nullable instancetype)initWithDate:(NSDate *)date
{
    if (!(self = [super init]))
    {
        return nil;
    }
    
    // It all starts with the Julian date
    NSCalendar *calendar = [[NSCalendar alloc] initWithCalendarIdentifier:NSCalendarIdentifierGregorian];
    calendar.timeZone = [NSTimeZone timeZoneForSecondsFromGMT:0];

    const NSCalendarUnit units = NSCalendarUnitYear |
                                 NSCalendarUnitMonth |
                                 NSCalendarUnitDay |
                                 NSCalendarUnitHour |
                                 NSCalendarUnitMinute |
                                 NSCalendarUnitSecond;
    NSDateComponents *components = [calendar components:units fromDate:date];
    if (!components)
    {
        return nil;
    }

    sun = std::make_unique<Sun>(components.year, components.month, components.day,
                                components.hour, components.minute, components.second);
    
    return self;
}

- (void)dealloc
{
    sun.reset();
}

- (MaplyCoordinate3d)direction
{
    if (sun)
    {
        const Point3d dir = sun->getDirection();
        return MaplyCoordinate3dMake(dir.x(), dir.y(), dir.z());
    }
    return kMaplyNullCoordinate3d;
}

- (MaplyCoordinate3d)position
{
    if (sun)
    {
        const auto height = 149.6 * 1000000 * 1000 / EarthRadius;
        return MaplyCoordinate3dMake(sun->sunLon,sun->sunLat, height);
    }
    return kMaplyNullCoordinate3d;
}

- (MaplyLight * _Nullable )makeLight
{
    return [self makeLightWithAmbient:0.1f diffuse:0.8f];
}

- (MaplyLight * _Nullable)makeLightWithAmbient:(float)ambient diffuse:(float)diffuse
{
    MaplyLight *light = [[MaplyLight alloc] init];
    const MaplyCoordinate3d dir = self.direction;
    light.pos = MaplyCoordinate3dMake(dir.x, dir.z, dir.y);
    light.ambient = [UIColor colorWithRed:ambient green:ambient blue:ambient alpha:1.0];
    light.diffuse = [UIColor colorWithRed:diffuse green:diffuse blue:diffuse alpha:1.0];
    light.viewDependent = true;
    return light;
}

@end
