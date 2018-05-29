# -*- coding: utf-8 -*-

import neato
import numpy as np
from utils import abs_alfa, for_each, print_each, separator
from collections import OrderedDict as odict

## DEFAULT SECTOR CONFIGURATIONS ##

def priorityConfiguration():
    """
    front_right  300/-60  ~ 330/-30  (30 º)
    front       330/-30  ~ 30       (60 º)
    front_left 30       ~ 60       (30 º)
    left       60       ~ 120      (60 º)
    back_left  120      ~ 150      (30 º)
    back        150      ~ 210/-150 (60 º)
    back_right   210/-150 ~ 240/-120 (30 º)
    right        240/-120 ~ 300/-60  (60 º)
    """
    return LaserSectors(
            LaserSector('front_right', -60, -30, front=True),
            LaserSector('front', -30, 30, front=True),
            LaserSector('front_left', 30, 60, front=True),
            LaserSector('left', 60, 120),
            LaserSector('back_left', 120, 150, back=True),
            LaserSector('back', 150, 210, back=True),
            LaserSector('back_right', 210, 240, back=True),
            LaserSector('right', 240, 300))

def commonConfiguration(offset=18):
    """
    Ten equal-sized divisions of 36º each
    """
    return LaserSectors(
            LaserSector('front_outer_right', -36*2 - offset, -36 - offset, front=True),
            LaserSector('front_center_right', -36 - offset, 0 - offset, front=True),
            LaserSector('front_center', 0 - offset, 36 - offset, front=True),
            LaserSector('front_center_left', 36 - offset, 36*2 - offset, front=True),
            LaserSector('front_outer_left', 36*2 - offset, 36*3 - offset, front=True),
            LaserSector('back_outer_left', 36*3 - offset, 36*4 - offset, back=True),
            LaserSector('back_center_left', 36*4 - offset, 36*5 - offset, back=True),
            LaserSector('back_center', 36*5 - offset, 36*6 - offset, back=True),
            LaserSector('back_center_right', 36*6 - offset, 36*7 - offset, back=True),
            LaserSector('back_outer_right', 36*7 - offset, 36*8 - offset, back=True))

def avoidingConfiguration(offset=18):
    """
    front: 18º 71º 2º 71º 18º
    back: 18º 71º 2º 71º 18º
    """
    return LaserSectors(
            LaserSector('front_outer_right', -90, -90 + 36, front=True),
            LaserSector('front_center_right', -54, -54 + 53, front=True),
            LaserSector('front_center', -1, -1 + 2, front=True),
            LaserSector('front_center_left', 1, 1 + 53, front=True),
            LaserSector('front_outer_left', 54, 54 + 36, front=True),
            LaserSector('back_outer_left', 90, 90 + 36, back=True),
            LaserSector('back_center_left', 126, 126 + 53, back=True),
            LaserSector('back_center', 179, 179 + 2, back=True),
            LaserSector('back_center_right', 181, 181 + 53, back=True),
            LaserSector('back_outer_right', 234, 234 + 36, back=True))

## LASER ##

class Laser(object):
    """ A rotative laser with 360 distance rays, one for each angle """

    OFFSET = 243 # mm (edge zero)

    def __init__(self, lines, configuration=priorityConfiguration):
        self.sectors = configuration()

        for line in lines: # each line is structured as "alfa,dist,intensity,errorCode"
            LaserRay(self, line.split(','), add=True)

        # sort sectors by alfa for better visualization
        for_each(self.sectors, LaserSector.sort)

        # add attributes (subscriptions) for every sector with their mean LaserRay
        self.apply(lambda sector: setattr(self, sector.tag, self.mean(sector.tag)))

    def all(self, tag=''):
        """ LaserRay array for every angle in sector tag, leave tag to default for all angles in all sectors """
        return self.sectors.rays(tag)

    def rays(self, empties=False):
        """ LaserRay array, average of every sector (only valid rays, or empties optionally) """
        return [self[tag] for tag in self.sectors.tags if empties or self[tag].is_valid()]

    def front_average(self):
        """ LaserRay, average of configuration front sectors, without errors """
        ray = _mean(self, self.sectors.rays_if(LaserSector.is_front))
        ray.tag = 'front_average'
        return ray

    def back_average(self):
        """ LaserRay, average of configuration back sectors, without errors """
        ray = _mean(self, self.sectors.rays_if(LaserSector.is_back))
        ray.tag = 'back_average'
        return ray

    def apply(self, block):
        """ Apply block function for each sector following configuration sector order """
        for_each(self.sectors, block)

    def filter(self, condition, tag=''):
        """ Get laser rays matching a condition, leave tag to default to return rays matching the condition from all sectors """
        return list(filter(condition, self.all(tag)))

    def farthest(self, average=True):
        """ Get the ray with farthest distance, either for average rays or for all rays """
        return self.__ray_by_dist(max, average)

    def nearest(self, average=True):
        """ Get the ray with nearest distance, either for average rays or for all rays """
        return self.__ray_by_dist(min, average)

    def __ray_by_dist(self, dist_f, average=True):
        rays = self.rays(empties=True) if average else self.all()
        distances = np.array(map(lambda ray: ray.dist, rays))
        match_dist = dist_f(distances)
        matches_indices = np.where(distances == match_dist)[0]
        match_rays = [rays[i] for i in matches_indices]
        if len(match_rays) == 1:
            return match_rays[0]
        larger_sectors = map(lambda ray: ray.sector.degrees(), match_rays)
        return match_rays[larger_sectors.index(max(larger_sectors))]

    def summary(self):
        """ Print mean rays for every non-empty valid sector, including front and back averages """
        print(separator())
        print_each(self.rays())
        print(self.front_average())
        print(self.back_average())

    def show(self, tag='', empties=True):
        """
        Print all rays for every angle and sector tag (leave to default for all sectors).
        If `empties` is False then only non-empty rays are shown.
        """
        print(separator())
        def print_sector(tag):
            print_each(self.filter(lambda ray: empties or not ray.is_empty(), tag))
        if tag != '':
            print_sector(tag)
        else:
            self.apply(lambda sector: print_sector(sector.tag))

    def mean(self, tag=''):
        """ LaserRay, average of #all(tag) valid rays """
        ray = _mean(self, self.all(tag))
        ray.tag = tag
        return ray

    def __getitem__(self, tag): # makes Laser subscriptable by sector tag
        return getattr(self, tag)

class LaserRay(object):
    """ A laser ray with an angle, distance, intensity and with possible error """

    DIST_LIMIT = 2000 # mm

    def __init__(self, laser, values, tag='', add=False):
        self.tag = tag
        self.laser = laser
        self.alfa = int(values[0]) # degrees
        self.original_dist = int(values[1]) # millimeters
        self.intensity = int(values[2])
        self.errorCode = int(values[3])

        if self.original_dist == 0 or self.is_limited():
            self.dist = LaserRay.DIST_LIMIT
        elif self.original_dist <= Laser.OFFSET:
            self.dist = 0
        else:
            self.dist = self.original_dist - Laser.OFFSET

        self.sector = laser.sectors.find(self.alfa)
        self.alfa = self.sector.normalize(self.alfa)

        if add:
            self.sector._force_add(self)

    def is_empty(self):
        """ True if laser did not detected anything with this ray or is limited, False otherwise """
        return self.dist == LaserRay.DIST_LIMIT

    def is_error(self):
        """ True if errorCode != 0, False otherwise """
        return self.errorCode != 0

    def is_valid(self):
        """ True iff not empty and not error and not limited """
        return not self.is_empty() and not self.is_error()

    def is_limited(self):
        """ True if original_dist - Laser.OFFSET >= DIST_LIMIT, False otherwise """
        return self.original_dist - Laser.OFFSET >= LaserRay.DIST_LIMIT

    def proximity(self):
        """ Inverse of distance ('in contact' means LaserRay.DIST_LIMIT is returned) """
        return LaserRay.DIST_LIMIT - self.dist

    def proximity_percent(self):
        """ Percent of proximity from 0 to 1 """
        return self.proximity() / float(LaserRay.DIST_LIMIT)

    def __repr__(self):
        s = "Alfa %s" % self.alfa
        if self.tag != '':
            s = '[' + self.tag.replace('_', ' ').upper() + ']' + (' ' * (self.laser.sectors.max_tag_size - len(self.tag))) + '\t' + s
        def append(title, value, suffix=''):
            if value != 0.0:
                return ', ' + title + ' ' + str(value) + suffix
            return ''
        s = s + append("Dist", float(self.original_dist) / 1000, suffix=' m' + (" LIMITED" if self.is_limited() else ''))
        s = s + append("Intensity", self.intensity)
        if self.is_error():
            s = s + append("ERROR", self.errorCode)
        return s

class LaserSector(object):
    """ Represents a division on the 360º circumference for rotative laser """

    def __init__(self, tag, start, end, front=False, back=False):
        assert not (front and back)
        assert end - start >= 0 and end - start <= 360
        self.tag = tag
        self.start = start # degrees inclusive
        self.end = end # degrees exclusive
        self.front = front
        self.back = back
        self.__rays = []

    def add(self, ray):
        """ Add `ray` if it pertains to this sector and returns if was added """
        if self.includes(ray.alfa):
            self._force_add(ray)
            return True
        return False

    def _force_add(self, ray):
        ray.tag = self.tag
        ray.alfa = self.normalize(ray.alfa)
        self.__rays.append(ray)

    def add_all(self, rays):
        """ Add all rays pertaining to this sector """
        for_each(rays, self.add)

    def includes(self, alfa):
        """ Check if `alfa` pertains to this sector """
        alfa = self.normalize(alfa)
        return alfa >= self.start and alfa < self.end

    def sort(self, key=lambda ray: ray.alfa):
        """ Sort rays with a `key` (default alfa) """
        self.__rays.sort(key=key)

    def rays(self):
        """ Get all rays in this sector """
        return self.__rays

    def degrees(self):
        """ Returns the angle in degrees of this sector """
        return self.end - self.start

    def is_front(self):
        """ Returns if this sector is part of the front """
        return self.front

    def is_back(self):
        """ Returns if this sector is part of the back """
        return self.back

    def center(self):
        """ Returns the mean angle of this sector """
        return abs_alfa((self.start + self.end)/2)

    def normalize(self, alfa):
        """ Get normalized `alfa` mod 360 so negative start value for this sector means alfa will be turned negative if |alfa| >= |start| """
        alfa = abs_alfa(alfa)
        return alfa - 360 if self.start < 0 and alfa >= abs_alfa(self.start) else alfa

class LaserSectors(object):
    """ Group of several sectors of rays """

    def __init__(self, *sectors):
        self.__sectors = odict()
        for sector in sectors:
            self.__sectors[sector.tag] = sector
        self.tags = self.__sectors.keys()
        self.max_tag_size = max(len(tag) for tag in self.tags)

    def __getitem__(tag): # subscriptable
        """ Get a sector identified by `tag`, returns None if does not exists """
        return self.__sectors.get(tag)

    # make sectors iterable
    def __iter__(self):
        return iter(self.__sectors.values())

    def find(self, alfa):
        """ Get sector in which alfa can be included """
        return next(sector for sector in self if sector.includes(alfa))

    def opposite(self, alfa):
        """ Get sector in which alfa + 180 can be included """
        return self.find(alfa + 180)

    def add(self, ray):
        """ Adds `ray` to the proper sector, which is returned """
        sector = self.find(ray.alfa)
        sector._force_add(ray)
        return sector

    def rays(self, tag=''):
        """ Get all rays in the sector identified by `tag`, leave `tag` by default for all sectors """
        if tag != '':
            return self.__sectors[tag].rays()
        rays = []
        for_each(self, lambda s: rays.extend(s.rays()))
        return rays

    def rays_if(self, sector_condition):
        """ Get all rays from sectors matching a condition """
        rays = []
        for_each(self.filter(sector_condition), lambda s: rays.extend(s.rays()))
        return rays

    def merge(self, tag, sector_condition):
        """ Create a new sector with all rays from sectors matching a condition """
        sectors = self.filter(sector_condition)
        result = LaserSector(tag,
                    start=min(map(lambda s: s.start, sectors)),
                    end=max(map(lambda s: s.end, sectors)),
                    front=all(sector.front for sector in sectors),
                    back=all(sector.back for sector in sectors))
        for_each(sectors, lambda s: result.add_all(s.rays()))
        return result

    def filter(self, condition):
        """ Get all sectors matching `condition` """
        return [sector for sector in self if condition(sector)]

    def all_front(self):
        """ A LaserSector with all rays in front sectors """
        return self.merge('ALL FRONT', LaserSector.is_front)

    def all_back(self):
        """ A LaserSector with all rays in back sectors """
        return self.merge('ALL BACK', LaserSector.is_back)

## GLOBAL METHODS ##

def _mean(laser, laser_rays):
    """ LaserRay, average of valid laser_rays """
    assert len(laser_rays) > 0

    alfa = 0
    dist = 0
    intensity = 0
    size = 0

    for ray in list(filter(LaserRay.is_valid, laser_rays)):
        size = size + 1
        alfa = alfa + ray.alfa
        dist = dist + ray.dist
        intensity = intensity + ray.intensity
    
    if size == 0:
        size = 1
        alfa = sum(map(lambda ray: ray.alfa, laser_rays))/len(laser_rays)

    return LaserRay(laser, [alfa / size, dist / size, intensity / size, 0])

def random_laser(configuration=priorityConfiguration):
    """ Builds a semi-random Laser, only for testing purposes """
    lines = []
    def random_error():
        return np.random.choice([0, 1], p=[0.9, 0.1]).item()
    def random_distance():
        #p=[0.93125, 0.0125, 0.025, 0.025, 0.00625]
        p=[0.85, 0.04, 0.06, 0.05]
        return np.random.choice([0, 300, 1500, LaserRay.DIST_LIMIT + 1], p=p).item()
    for alfa in range(360):
        lines.append('%i,%i,0,%i' % (alfa, random_distance(), random_error()))
    return Laser(lines, configuration)