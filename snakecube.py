#!/usr/bin/env python3


import copy
import logging

from vector import (Vector3D, baseVectors, baseDirections, rotx, roty, rotz,
                    multiply, mirror_yz, mirror_zx, mirror_xy)


logging.basicConfig(format='%(levelname)s:%(name)s: %(message)s',
                    level=logging.WARNING)
logger = logging.getLogger(__name__)


def _makeJointDirections():
    """Generate a lookup table for the backtracking head.

    It contains lists of all possible new directions at a joint element.  When
    the backtracking head encounters a joint element, it can lookup possible
    new directions in this dict.  With our snake cube we only have one
    interesting kind of joint element (the 90 degree joint), thus one lookup
    table suffices.

    Returns:
        dict: The generated lookup table.

        key = The current direction.
        value = The list of all possible new directions for a given key.

    """
    # number of base directions (base vectors and their inverse) in a 3D plane
    N_PLANE_BASEDIR = 4
    # number of base vectors in 3D space
    N_BASEVEC = 3

    mapVtoRM = dict()
    mapVtoRM[str(Vector3D(1,0,0))] = rotx
    mapVtoRM[str(Vector3D(0,1,0))] = roty
    mapVtoRM[str(Vector3D(0,0,1))] = rotz

    jointDirections = dict()
    for i, vector in enumerate(baseVectors):
        newDirections = []
        rotmat = mapVtoRM[str(vector)]
        # add one other (random) base vector which is perpendicular ...
        newDirections.append(baseVectors[(i+1)%N_BASEVEC])
        # ... and include all other base directions in that plane (+/-)
        for j in range(N_PLANE_BASEDIR):
            newDirections.append(multiply(rotmat, newDirections[-1]))
        jointDirections[str(vector)] = newDirections
        jointDirections[str(-1*vector)] = newDirections

    return jointDirections


jointDirections = _makeJointDirections()
"""A lookup table for the backtracking head containing new joint directions.

See documentation for _makeJointDirections() for more info.
"""

# number of unique joint states for the 90 degree dual-joint element
N_JNTS = 4

# possible joint states for the 90 degree dual-joint element
JOINT0 = 0
JOINT1 = 1
JOINT2 = 2
JOINT3 = 3

# cube: position occupied by some other chain element
POS_USED = -1
# cube: position free
POS_FREE = -2


class BacktrackHead:
    """A class implementing the "head" (or tip) of the backtracking algorithm.

    It basically consists of two vectors, one describing the current position
    of the head, the other describing the direction in which the head is
    "looking".  In general, the direction vector and "joint state" mean the
    same thing here.

    Args:
        base (Vector3D): Current position of the backtracking head.
        direction (Vector3D): Current direction of the backtracking head.
          Possible values are b and (-b), where b is one of the three cartesian
          unit vectors (only unit vectors are supported as base vectors).

    Attributes:
        base (Vector3D): Current position of the backtracking head.
        direction (Vector3D): Current direction of the backtracking head.

    """
    __slots__ = ['base', 'direction', '_old_direction']

    def __init__(self, base, direction):
        self.base = base
        self.direction = direction
        self._old_direction = None

    def __repr__(self):
        return f"BacktrackHead({self.base!r}, {self.direction!r})"

    def get_sign(self):
        """Return the current sign of the backtracking head's direction."""
        if max(self.direction.to_list()) == 1:
            return 1
        else:
            return -1

    def move(self, nsteps):
        """Move the backtracking head nsteps forward.

        Move the backtracking head's base vector nsteps steps into the
        direction of the backtracking head's direction vector.

        Args:
            nsteps (int): The number of steps the backtracking head shall move.

        """
        self.base += nsteps * self.direction

    def change_direction(self):
        """Change the backtracking head's direction by 90 degree (new joint).

        Change the backtracking head's direction vector to a new direction
        vector parallel to one of the base axes so that the new direction
        vector is perpendicular to the current direction vector.  This
        typically occurs after moving the head forward, when a new joint
        element is encountered. The new direction vector (i.e. the state of the
        joint) is chosen arbitrarily to the first of the four possible values.

        The current direction vector is saved in the internal variable
        self._old_direction prior to the change, which is used by
        self.rotate_to().

        """
        self._old_direction = self.direction
        joint_state = JOINT0
        self.direction = jointDirections[str(self._old_direction)][joint_state]

    def rotate_to(self, joint_state):
        """Change the backtracking head's direction by 90 degree (same joint).

        Change the backtracking head's direction vector to a new direction
        vector parallel to one of the base axes so that the new direction
        vector is perpendicular (1) to the current direction vector and also
        (2) to the old direction vector self._old_direction as saved by
        self.change_direction().

        This typically is done when the backtracking algorithm is "backing up"
        (going backwards) because there are no (or no more) solutions with the
        current joint state at the backtracking head.

        The new direction vector (i.e. the state of the joint) is explicitly
        specified by the argument joint_state.

        Args:
            joint_state (int): The new state of the joint element.

            The value should be specified using one of the constants JOINTx,
            where x is a number in range(N_JNTS).

        """
        assert self._old_direction is not None
        self.direction = jointDirections[str(self._old_direction)][joint_state]


class Backtrack:
    """A class containing the core algorithm for backtracking the snake cube.

    After creating an object of this class, the backtracking problem can be
    solved by simply calling self.solve() on it, which returns self._paths,
    containing all possible solutions.

    Args:
        chain (list(int)): Representation of the snake cube chain to be solved.
        cubesize (int): Edge length of the snake cube.
        head (BacktrackHead): Starting position for the backtracking algorithm.

    Attributes:
        chain (list(int)): Representation of the snake cube chain to be solved.
        cubesize (int): Edge length of the snake cube.
        head (BacktrackHead): Current position of the backtracking algorithm.
        _pos (int): Current position in the chain. Starts at 0.
        _chainlength (int): Length of the chain.
        _cube (list(list(list(int)))): A matrix representing the snake cube.
          The matrix represents the available space in the cube that can be
          "occupied" by the chain.

          Each field in the matrix is set one of the following values:

            JOINTx (where x is a number in range(N_JNTS)): The value indicates
              the state of the 90 degree dual-joint element which is currently
              present at this field.
            POS_USED: The field is currently occupied by another chain element.
            POS_FREE: The field is unoccupied and free for use.
        _paths (list(list(int))): List of the solutions so far found.
          Each element in this variable represents a solution, i.e. a "chain
          folding" that "fits" the chain into the cube.

    """

    __slots__ = ['chain', 'cubesize', 'head', '_pos', '_chainlength', '_cube',
                 '_paths']

    def __init__(self, chain, cubesize, head):
        self.chain = chain
        self.cubesize = cubesize
        self.head = head
        self._pos = 0
        self._chainlength = len(chain)
        self._cube = [[[POS_FREE for i in range(cubesize)]
                       for j in range(cubesize)] for k in range(cubesize)]
        self._paths = []

    def solve(self):
        """Run backtracking algorithm in a loop until all solutions are found.
        
        This method calls self._backtrack() until it returns no more solutions.
        All solutions are gathered in the internal variable self._paths, which
        is returned when there are no more solutions to be found.

        Returns:
            list(list(int)): List of all possible solutions.

        """
        if self._paths == []:
            path = self._backtrack()
            while path is not None:
                logger.info('new solution found: %s', path)
                self._paths.append(path)
                path = self._backtrack()
            logger.info('backtracking exhausted (no more solutions for the '
                        'specified starting point and direction)')
        else:
            # already solved
            pass
        return self._paths

    def _joint_wrapped(self, joint_state):
        """Check if a given joint "wrapped around".

        There are N_JNTS (4) possible states for a 90 degree joint, which all
        have to be tried during the backtracking. When a given joint wrapps
        around (i.e.  it takes a value which was already taken earlier), the
        algorithm should "back up" (go backwards) and change the join prior to
        the current joint to find any new solutions.

        Returns:
            bool: True if joint wrapped around, False otherwise.

        """
        return joint_state >= N_JNTS

    def _backtrack(self):
        """Core implementation of the backtracking algorithm.

        The method instantly returns if it finds a new solution, returning that
        solution. When it is called again, it will resume the backtracking
        process from where it stopped to find another new solution. This
        continues until there are no more new solutions available, at which
        point None is returned, signalling that the backtracking process is
        exhausted.

        Returns:
            list(int)/None: A new solution to the backtracking problem, or None
            if no more solutions are available, i.e. the backtracking is
            exhausted.

        """
        if self._pos == 0:          # first run
            logger.info('>> starting backtracking...')
            path = []
            self._cube_set_offset(0, POS_USED)
        else:                       # subsequent run
            logger.info('>> restarting backtracking...')
            path = copy.deepcopy(self._paths[-1])

            # restore _pos, head, and path
            self._pos -= 1
            self.head = path.pop()

            # restore _cube
            steps_to_delete = self.chain[self._pos]
            for i in range(1, steps_to_delete + 1):
                self._cube_set_offset(i, POS_FREE)

            # pick new direction
            joint_state = self._cube_get_offset(0)
            self.head.rotate_to((joint_state+1)%N_JNTS)
            self._cube_set_offset(0, joint_state+1)
            logger.debug('>> going back to pos %s...', self._pos)

        debug_counter = 0
        while self._pos < self._chainlength:
            # In every cycle, self._pos points to that slice in self.chain
            # which has no yet been put into path (the current solution), i.e.
            # the slice has not yet been "walked over". Note that after a slice
            # has been walked over and recorded into path (in form of a record
            # of the backtracking head's state), it can be removed therefrom
            # again.
            #
            # The backtracking head self.head determines the position of the
            # current slice in the cube. head.base points to the beginning of
            # the current slice (== end of the last slice), and head.direction
            # points to the end of the current slice (== start of the next
            # slice).
            logger.debug(f'======== CYCLE NO: {debug_counter} ========')
            debug_counter += 1
            logger.debug('_pos: %d', self._pos)
            logger.debug('path: %s', path)
            logger.debug('head: %s', self.head)
            joint_state = self._cube_get_offset(0)
            logger.debug('joint_state: %s', joint_state)
            if not self._joint_wrapped(joint_state):    # joint_state okay
                logger.debug('>> joint okay: joint not wrapped')
                steps_needed = self.chain[self._pos]
                steps_allowed = self._nsteps()
                logger.debug('steps_needed: %s', steps_needed)
                logger.debug('steps_allowed: %s', steps_allowed)
                if steps_allowed < steps_needed:        # the way is free...
                    logger.debug('>> steps_allowed < steps_needed')
                    try:
                        self.head.rotate_to((joint_state+1)%N_JNTS)
                    except AssertionError:
                        # This happens when the backtracking head tries to
                        # rotate before any forward movement, i.e. at the very
                        # beginning of the backtracking process. In this case,
                        # there is no valid solution for the specified
                        # backtracking head (starting point + direction).
                        return None
                    self._cube_set_offset(0, joint_state + 1)
                    logger.debug('>> trying new joint %s of direction %s '
                                 'which maps to %s', joint_state + 1,
                                 self.head._old_direction.to_list(),
                                 self.head.direction.to_list())
                else:   # the way is not free...
                    logger.debug('>> moving %d steps forward...', steps_needed)
                    new_joint_state = JOINT0

                    # cube: record new joint direction and set fields between
                    # joints to POS_USED, indicating occupation
                    for i in range(1, steps_needed):
                        self._cube_set_offset(i, POS_USED)
                    self._cube_set_offset(steps_needed, new_joint_state)

                    # path: record current state of head
                    path.append(copy.deepcopy(self.head))

                    self.head.move(steps_needed)
                    self.head.change_direction()
                    self._pos += 1
                    logger.debug('>> trying new joint %s of direction %s '
                                 'which maps to %s', new_joint_state,
                                 self.head._old_direction.to_list(),
                                 self.head.direction.to_list())
            else:   # joint_state wrapped around
                logger.debug('>> joint wrapped around! joint_state >= %d',
                             N_JNTS)
                if self._pos == 1:
                    # backtracking exhausted (no more solutions for the
                    # specified starting point and direction)
                    return None

                # restore _pos, head, and path
                self._pos -= 1
                self.head = path.pop()

                # restore cube
                steps_to_delete = self.chain[self._pos]
                for i in range(1, steps_to_delete + 1):
                    self._cube_set_offset(i, POS_FREE)

                # pick new direction
                joint_state = self._cube_get_offset(0)
                self.head.rotate_to((joint_state+1)%N_JNTS)
                self._cube_set_offset(0, joint_state+1)
                logger.debug('>> going back to pos %s...', self._pos)

        # path now contains len(self.chain) copies of the backtracking head
        # (BacktrackHead objects). Each element's head attribute points to the
        # beginning of a slice, and each element's direction attribute points
        # to the beginning of the next slice.
        return path

    def _cube_get_offset(self, offset):
        """Return the value at the backtracking head after moving offset steps.

        Args:
            offset (int): Number of steps to make before retrieving the value.

        Returns:
            int: The value in the internal cube matrix at the postion of the
            backtracking head, after walking offset steps into the current
            direction of the backtracking head.

        """
        pos = self.head.base + (offset * self.head.direction)
        return self._cube[pos.x][pos.y][pos.z]

    def _cube_set_offset(self, offset, value):
        """Set the value at the backtracking head after moving offset steps.

        Args:
            offset (int): Number of steps to make before setting the value.
            value (int): The value to be set in the internal cube matrix at the
              postion of the backtracking head, after walking offset steps into
              the current direction of the backtracking head.

        """
        pos = self.head.base + (offset * self.head.direction)
        self._cube[pos.x][pos.y][pos.z] = value

    def _nsteps(self):
        """Return the number of valid steps the backtracking head could make.

        Returns:
            int: The current number of straight steps the backtracking head can
            make, while (1) staying within the cube's bounds and (2) not
            colliding with any "previous" parts of the chain. Depends on
            self._cube, self.head, and self.cubesize.

        """
        nsteps = 0
        pos = self.head.base
        if self.head.get_sign() == 1:
            while ((max((pos + self.head.direction).to_list()) < self.cubesize)
                   and (self._cube_get_offset(nsteps+1) == POS_FREE)):
                pos += self.head.direction
                nsteps += 1
        else:   # self.head.get_sign() == -1
            while ((min((pos + self.head.direction).to_list()) >= 0)
                   and (self._cube_get_offset(nsteps+1) == POS_FREE)):
                pos += self.head.direction
                nsteps += 1
        return nsteps


def main():
    """Demonstrate usage of the Backtrack class with an example chain."""
    logger.debug('jointDirections:')
    for key in jointDirections:
        logger.debug("\t%s: %s", key, jointDirections[key])

    # Unique (except for start/end orientation) representation of the chain.
    # Every element is assigned a number, based on the number and configuration
    # of its joints.
    # 0 = element with only one joint (start/end of the chain)
    # 1 = element with two joints on opposite sides (straight joint element)
    # 2 = element with two joints on adjacent sides (90 degree joint element)
    chain_elements = [0, 1, 2, 2, 2, 1, 2, 2, 1, 2, 2, 2, 1, 2, 1, 2, 2, 2, 2,
                      1, 2, 1, 2, 1, 2, 1, 0]

    # Unique (except for start/end orientation) representation of the chain.
    # Every slice is assigned a number, based on its length. All slices are
    # assumed to be connected by the same type of joint: a 90 degree dual-joint
    # element. Every 90 degree dual-joint element is part of two slices.
    # 1 = slice with 2 elements (backtracking head can be incremented by 1).
    # 2 = slice with 3 elements (backtracking head can be incremented by 2).
    chain_slices = [2, 1, 1, 2, 1, 2, 1, 1, 2, 2, 1, 1, 1, 2, 2, 2, 2]

    # Use a 3x3x3 cube.
    cubesize = 3

    #
    # Backtracking
    #

    # These are all the interesting starting points and directions.
    btheads = [
        BacktrackHead(Vector3D(0, 0, 0), Vector3D(1, 0, 0)),
        BacktrackHead(Vector3D(1, 0, 0), Vector3D(1, 0, 0)),
        BacktrackHead(Vector3D(1, 0, 0), Vector3D(0, 1, 0)),
        BacktrackHead(Vector3D(1, 1, 0), Vector3D(1, 0, 0)),
        BacktrackHead(Vector3D(1, 1, 0), Vector3D(0, 0, 1)),
        BacktrackHead(Vector3D(1, 1, 1), Vector3D(1, 0, 0)),
        ]

    all_solutions = []
    for bthead in btheads:
        backtrack = Backtrack(chain_slices, cubesize, bthead)
        print(f'>> start backtracking with starting point {bthead}')
        solutions = backtrack.solve()
        # print solutions
        print('==== solutions ====')
        for i, path in enumerate(solutions):
            all_solutions.append(path)
            path_bases = list(map(lambda x: x.base.to_list(), path))
            path_directions = list(map(lambda x: x.direction.to_list(), path))
            print(f'solution {i} (as base points): {path_bases}')
            #print(f'solution {i} (as directions): {path_directions}')
        if len(solutions) == 0:
            print('no solutions')
        print()

    # Demonstrate that the only two solutions found are similar.
    path0points = list(map(lambda x: x.base.to_list(), all_solutions[0]))
    path1points = list(map(lambda x: x.base.to_list(), all_solutions[1]))
    f = lambda x: multiply(rotz, multiply(rotz, multiply(rotx,
                  multiply(mirror_yz, x)))).to_list()
    if path1points == list(map(f, path0points)):
        print('The two solutions are similar! Just mirror one solution at the '
              'yz-plane, rotate 90 degree about the x axis and 180 degree '
              'about the z axis, and you got the other solution.')


if __name__ == '__main__':
    main()

