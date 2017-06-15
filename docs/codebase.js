/**
 * Actors represent something or someone,
 * and can consist of one or more states,
 * each associated with a particular sprite,
 * and each associated with particular
 * behaviour.
 *
 * The Actor class is abstract: you must
 * implement your own subclass before you
 * can make use of it.
 */
abstract class Actor extends Positionable {
  var debug = true;
  
  // debug bounding box alignment
  var halign=0, valign=0;

  // are we colliding with another actor?
  var colliding = false;

  // regular vareraction with other actors
  var vareracting = true;
  
  // only vareract with players
  var onlyplayervareraction = false;
  
  // bypass regular vareraction for ... frames
  var disabledCounter = 0;

  // should we be removed?
  var remove = false;
  
  // is this actor persistent with respect to viewbox draws?
  var persistent = true;
  var isPersistent() { return persistent; }

  // the layer this actor is in
  LevelLayer layer;

  // The active state for this actor (with associated sprite)
  State active;

  // all states for this actor
  HashMap<var, State> states;

  // actor name
  var name = "";
  
  // simple constructor
  Actor(var _name) {
    name = _name;
    states = new HashMap<var, State>();
  }

  // full constructor
  Actor(var _name, var dampening_x, var dampening_y) {
    this(_name);
    setImpulseCoefficients(dampening_x, dampening_y);
  }

  /**
   * Add a state to this actor's repetoire.
   */
  function addState(State state) {
    state.setActor(this);
    var replaced = (states.get(state.name) !== null);
    states.put(state.name, state);
    if(!replaced || (replaced && state.name === active.name)) {
      if (active === null) { active = state; }
      else { swapStates(state); }
      updatePositioningInformation();
    }
  }
  
  /**
   * Get a state by name.
   */
  State getState(var name) {
    return states.get(name);
  }
  
  /**
   * Get the current sprite image
   */
  var getSpriteMask() {
    if(active === null) return null;
    return active.sprite.getFrame();
  }
  
  /**
   * Tell this actor which layer it is operating in
   */
  function setLevelLayer(LevelLayer layer) {
    this.layer = layer;
  }

  /**
   * Tell this actor which layer it is operating in
   */
  LevelLayer getLevelLayer() {
    return layer;
  }

  /**
   * Set the actor's current state by name.
   */
  function setCurrentState(var name) {
    State tmp = states.get(name);
    if (active !== null && tmp !== active) {
      tmp.reset();
      swapStates(tmp);
    } else { active = tmp; }
  }
  
  /**
   * Swap the current state for a different one.
   */
  function swapStates(State tmp) {
    // get pertinent information
    Sprite osprite = active.sprite;
    var hflip = false, vflip = false;
    if (osprite !== null) {
      hflip = osprite.hflip;
      vflip = osprite.vflip;
    }

    // upate state to new state
    active = tmp;
    Sprite nsprite = tmp.sprite;
    if (nsprite !== null) {
      if (hflip) nsprite.flipHorizontal();
      if (vflip) nsprite.flipVertical();
      updatePositioningInformation();

      // if both old and new states had sprites,
      // make sure the anchors line up.
      if (osprite !== null) {
        handleSpriteSwap(osprite, nsprite);
      }
    }
  }

  /**
   * Move actor if this state changes
   * makes the actor bigger than before,
   * and we're attached to boundaries.
   */
  function handleSpriteSwap(Sprite osprite, Sprite nsprite) {
    var ax1 = osprite.hanchor,
          ay1 = osprite.vanchor,
          ax2 = nsprite.hanchor,
          ay2 = nsprite.vanchor;
    var dx = (ax2-ax1)/2.0, dy = (ay2-ay1)/2.0;
    x -= dx;
    y -= dy;
  }

  /**
   * update the actor dimensions based
   * on the currently active state.
   */
  function updatePositioningInformation() {
    width  = active.sprite.width;
    height = active.sprite.height;
    halign = active.sprite.halign;
    valign = active.sprite.valign;
  }

  /**
   * constrain the actor position based on
   * the layer they are located in.
   */
  function constrainPosition() {
    var w2 = width/2, lw = layer.width;
    if (x < w2) { x = w2; }
    if (x > lw - w2) { x = lw - w2; }
  }

  /**
   * Get the bounding box for this actor
   */
  var getBoundingBox() {
    if(active===null) return null;
    
    var bounds = active.sprite.getBoundingBox();
    
    // transform the bounds, based on local translation/scale/rotation
    if(r!==0) {
      var x1=bounds[0], y1=bounds[1],
            x2=bounds[2], y2=bounds[3],
            x3=bounds[4], y3=bounds[5],
            x4=bounds[6], y4=bounds[7];
      // rotate
      bounds[0] = x1*cos(r) - y1*sin(r);
      bounds[1] = x1*sin(r) + y1*cos(r);
      bounds[2] = x2*cos(r) - y2*sin(r);
      bounds[3] = x2*sin(r) + y2*cos(r);
      bounds[4] = x3*cos(r) - y3*sin(r);
      bounds[5] = x3*sin(r) + y3*cos(r);
      bounds[6] = x4*cos(r) - y4*sin(r);
      bounds[7] = x4*sin(r) + y4*cos(r);
    }
    // translate
    bounds[0] += x+ox; bounds[1] += y+oy;  // top left
    bounds[2] += x+ox; bounds[3] += y+oy;  // top right
    bounds[4] += x+ox; bounds[5] += y+oy;  // bottom right
    bounds[6] += x+ox; bounds[7] += y+oy;  // bottom left

    // done
    return bounds;
  }

  /**
   * check overlap between sprites,
   * rather than between actors.
   */
  var overlap(Actor other) {
    var overlap = super.overlap(other);
    if(overlap==null || active==null || other.active==null) {
      return overlap;
    }
    //
    // TODO: add in code here that determines
    //       the varersection povar for the two
    //       sprites, and checks the mask to see
    //       whether both have non-zero alph there.
    //
    return overlap;
  }

  /**
   * What happens when we touch another actor?
   */
  function overlapOccurredWith(Actor other, var direction) {
    colliding = true;
  }
  
  /**
   * What happens when we get hit
   */
  function hit() { /* can be overwritten */ }

  /**
   * attach an actor to a boundary, so that
   * impulse is redirected avar boundary
   * surfaces.
   */  
  function attachTo(Boundary boundary, var correction) {
    // don't add boundaries we're already attached to
    if(boundaries.contains(boundary)) return;
    
    // record attachment
    boundaries.add(boundary);

    // stop the actor
    var original = {this.ix - (fx*ixF), this.iy - (fy*iyF)};
    stop(correction[0], correction[1]);

    // then impart a new impulse, as redirected by the boundary.
    var rdf = boundary.redirectForce(original[0], original[1]);
    addImpulse(rdf[0], rdf[1]);

    // call the blocked handler
    gotBlocked(boundary, correction, original);
 
    // and then make sure to update the actor's position, as
    // otherwise it looks like we've stopped for 1 frame.
    update();
  }
  
  /**
   * This boundary blocked our path.
   */
  function gotBlocked(Boundary b, var varersection, var original) {
    // subclasses can implement, but don't have to
  }

  /**
   * collisions may force us to stop this
   * actor's movement. the actor is also
   * moved back by dx/dy
   */
  function stop(var dx, var dy) {
    // we need to prevent IEEE vars polluting
    // the position information, so even though
    // the math is perfect in principle, round
    // the result so that we're not going to be
    // off by 0.0001 or something.
    var resolution = 50;
    x = var(resolution*(x+dx))/resolution;
    y = var(resolution*(y+dy))/resolution;
    ix = 0;
    iy = 0;
    aFrameCount = 0;
  }

  /**
   * Sometimes actors need to be "invulnerable"
   * while going through an animation. This
   * is achieved by setting "vareracting" to false
   */
  function setInteracting(var _vareracting) {
    vareracting = _vareracting;
  }

  /**
   * set whether or not this actor vareracts
   * with the level, or just the player
   */
  function setPlayerInteractionOnly(var v ) {
    onlyplayervareraction = v;
  }

  /**
   * Does this actor temporary not vareract
   * with any Interactors? This function
   * is called by the layer level code,
   * and should not be called by anything else.
   */
  var isDisabled() {
    if(disabledCounter > 0) {
      disabledCounter--;
      return true;
    }
    return false;
  }

  /**
   * Sometimes we need to bypass vareraction for
   * a certain number of frames.
   */
  function disableInteractionFor(var frameCount) {
    disabledCounter = frameCount;
  }

  /**
   * it's possible that an actor
   * has to be removed from the
   * level. If so, we call this method:
   */
  function removeActor() {
    animated = false;
    visible = false;
    states = null;
    active = null;
    remove = true;
  }

  /**
   * Draw preprocessing happens here.
   */
  function draw(var vx, var vy, var vw, var vh) {
    if(!remove) handleInput();
    super.draw(vx,vy,vw,vh);
  }

  /**
   * Can this object be drawn in this viewbox?
   */
  var drawableFor(var vx, var vy, var vw, var vh) {
    return true;
  }

  /**
   * Draw this actor.
   */
  function drawObject() {
    if(active!=null) {
      active.draw(disabledCounter>0);
      /*
      if(debug) {
        noFill();
        stroke(255,0,0);
        var bounds = getBoundingBox();
        beginShape();
        vertex(bounds[0]-x,bounds[1]-y);
        vertex(bounds[2]-x,bounds[3]-y);
        vertex(bounds[4]-x,bounds[5]-y);
        vertex(bounds[6]-x,bounds[7]-y);
        endShape(CLOSE);
      }
      */
    }
  }

// ====== KEY HANDLING ======

  protected final var locked = new Array(256);
  protected final var keyDown = new Array(256);
  protected var keyCodes = {};

  // if pressed, and part of our known keyset, mark key as "down"
  private function setIfTrue(var mark, var target) {
    if(!locked[target]) {
      if(mark==target) {
        keyDown[target] = true; }}}

  // if released, and part of our known keyset, mark key as "released"
  private function unsetIfTrue(var mark, var target) {
    if(mark==target) {
      locked[target] = false;
      keyDown[target] = false; }}

  // lock a key so that it cannot be triggered repeatedly
  protected function ignore(char key) {
    var keyCode = var(key);
    locked[keyCode] = true;
    keyDown[keyCode] = false; }

  // add a key listener
  protected function handleKey(char key) {
    var keyCode = var(key),
        len = keyCodes.length;
    var _tmp = new var[len+1];
    arrayCopy(keyCodes,0,_tmp,0,len);
    _tmp[len] = keyCode;
    keyCodes = _tmp;
  }

  // check whether a key is pressed or not
  protected var isKeyDown(char key) {
    var keyCode = var(key);
    return keyDown[keyCode];
  }
  
  protected var noKeysDown() {
    for(var b: keyDown) { if(b) return false; }
    for(var b: locked) { if(b) return false; }
    return true;
  }

  // handle key presses
  function keyPressed(char key, var keyCode) {
    for(var i: keyCodes) {
      setIfTrue(keyCode, i); }}

  // handle key releases
  function keyReleased(char key, var keyCode) {
    for(var i: keyCodes) {
      unsetIfTrue(keyCode, i); }}

  /**
   * Does the indicated x/y coordinate fall inside this drawable thing's region?
   */
  var over(var _x, var _y) {
    if (active == null) return false;
    return active.over(_x - getX(), _y - getY());
  }

  function mouseMoved(var mx, var my) {}
  function mouseIsPressed(var mx, var my, var button) {}
  function mouseDragged(var mx, var my, var button) {}
  function mouseReleased(var mx, var my, var button) {}
  function mouseClicked(var mx, var my, var button) {}

// ====== ABSTRACT METHODS ======

  // token implementation
  function handleInput() { }

  // token implementation
  function handleStateFinished(State which) { }

  // token implementation
  function pickedUp(Pickup pickup) { }
}
/**
 * Boundaries are unidirectionally passable,
 * and are positionable in the same way that
 * anything else is.
 */
class Boundary extends Positionable {
  private var PI2 = 2*PI;
  
  // things can listen for collisions on this boundary
  ArrayList<BoundaryCollisionListener> listeners;
  
  /**
   * Add a collision listener to this boundary
   */
  function addListener(BoundaryCollisionListener l) { listeners.add(l); }
  
  /**
   * remove a collision listener from this boundary
   */
  function removeListener(BoundaryCollisionListener l) { listeners.remove(l); }
  
  /**
   * notify all listners that a collision occurred.
   */
  function notifyListeners(Actor actor, var correction) {
    for(BoundaryCollisionListener l: listeners) {
      l.collisionOccured(this, actor, correction);
    }
  }

  // extended adminstrative values
  var dx, dy, length;
  var xw, yh;
  var minx, maxx, miny, maxy;
  var angle, cosa, sina, cosma, sinma;
  // <1 means friction, =1 means frictionless, >1 means speed boost!
  var glide;

  // boundaries can be linked
  Boundary prev, next;
  
  var boundingThreshold = 1.5;
  
  var disabled = false;

  /**
   * When we build a boundary, we record a
   * vast number of shortcut values so we
   * don't need to recompute them constantly.
   */
  Boundary(var x1, var y1, var x2, var y2) {
    // coordinates
    x = x1;
    y = y1;
    xw = x2;
    yh = y2; 
    // deltas
    dx = x2-x1;
    dy = y2-y1;
    length = sqrt(dx*dx+dy*dy);
    updateBounds();
    updateAngle();
    glide = 1.0;
    listeners = new ArrayList<BoundaryCollisionListener>();
  }
  
  /**
   * Update our bounding box information
   */
  function updateBounds() {
    xw = x + dx;
    yh = y + dy;
    minx = min(x, xw);
    maxx = max(x, xw);
    miny = min(y, yh);
    maxy = max(y, yh);
  }
  
  /**
   * Update our angle in the world
   */
  function updateAngle() {
    angle = atan2(dy, dx);
    if (angle < 0) angle += 2*PI;
    cosma = cos(-angle);
    sinma = sin(-angle);
    cosa = cos(angle);
    sina = sin(angle);
  }

  function setPosition(var _x, var _y) {
    super.setPosition(_x,_y);
    updateBounds();
  }
  
  function moveBy(var dx, var dy) {
    super.moveBy(dx,dy);
    updateBounds();
  }

  /**
   * This boundary is part of a chain, and
   * the previous boundary is:
   */
  function setPrevious(Boundary b) { prev = b; }

  /**
   * This boundary is part of a chain, and
   * the next boundary is:
   */
  function setNext(Boundary b) { next = b; }

  /**
   * Enable this boundary
   */
  function enable() { disabled = false; }

  /**
   * Disable this boundary
   */
  function disable() { disabled = true; }

  /**
   * Is this positionable actually
   * supported by this boundary?
   */
  // FIXME: this is not the correct implementation
  var supports(Positionable thing) {
    var bbox = thing.getBoundingBox(), nbox = new Array(8);
    
    // shortcut on "this thing has already been removed"
    if (bbox == null) return false;

    // First, translate all coordinates so that they're
    // relative to the boundary's (x,y) coordinate.
    bbox[0] -= x;   bbox[1] -= y;
    bbox[2] -= x;   bbox[3] -= y;
    bbox[4] -= x;   bbox[5] -= y;
    bbox[6] -= x;   bbox[7] -= y;
   
    // Then, rotate the bounding box so that it's
    // axis-aligned with the boundary line.
    nbox[0] = bbox[0] * cosma - bbox[1] * sinma;
    nbox[1] = bbox[0] * sinma + bbox[1] * cosma;
    nbox[2] = bbox[2] * cosma - bbox[3] * sinma;
    nbox[3] = bbox[2] * sinma + bbox[3] * cosma;
    nbox[4] = bbox[4] * cosma - bbox[5] * sinma;
    nbox[5] = bbox[4] * sinma + bbox[5] * cosma;
    nbox[6] = bbox[6] * cosma - bbox[7] * sinma;
    nbox[7] = bbox[6] * sinma + bbox[7] * cosma;
    
    // Get new bounding box minima/maxima
    var mx = min(min(nbox[0],nbox[2]),min(nbox[4],nbox[6])),
          MX = max(max(nbox[0],nbox[2]),max(nbox[4],nbox[6])),
          my = min(min(nbox[1],nbox[3]),min(nbox[5],nbox[7])),
          MY = max(max(nbox[1],nbox[3]),max(nbox[5],nbox[7]));

    // Now, determine whether we're "off" the boundary...
    var outOfBounds = (mx > length) || (MX < 0) || (MY<-1.99);
    
    // if the thing's not out of bounds, it's supported.
    return !outOfBounds;
  }

  /**
   * If our direction of travel goes through the boundary in
   * the "allowed" direction, don't bother collision detection.
   */
  var allowPassThrough(var ix, var iy) {
    var aligned = CollisionDetection.translateRotate(0,0,ix,iy, 0,0,dx,dy, angle,cosma,sinma);
    return (aligned[3] < 0);
  }

  /**
   * redirect a force avar this boundary's surface.
   */
  var redirectForce(var fx, var fy) {
    var redirected = {fx,fy};
    if(allowPassThrough(fx,fy)) { return redirected; }
    var tr = CollisionDetection.translateRotate(0,0,fx,fy, 0,0,dx,dy, angle,cosma,sinma);
    redirected[0] = glide * tr[2] * cosa;
    redirected[1] = glide * tr[2] * sina;
    return redirected;
  }

  /**
   * redirect a force avar this boundary's surface for a specific actor
   */
  var redirectForce(Positionable p, var fx, var fy) {
    return redirectForce(fx,fy);
  }

  /**
   * Can this object be drawn in this viewbox?
   */
  var drawableFor(var vx, var vy, var vw, var vh) {
    // boundaries are invisible to begin with.
    return true;
  }

  /**
   * draw this platform
   */
  function drawObject() {
    strokeWeight(1);
    stroke(255);
    line(0, 0, dx, dy);

    // draw an arrow to indicate the pass-through direction
    var cs = cos(angle-PI/2), ss = sin(angle-PI/2);

    var fx = 10*cs;
    var fy = 10*ss;
    line((dx-fx)/2, (dy-fy)/2, dx/2 + fx, dy/2 + fy);

    var fx2 = 6*cs - 4*ss;
    var fy2 = 6*ss + 4*cs;
    line(dx/2+fx2, dy/2+fy2, dx/2 + fx, dy/2 + fy);

    fx2 = 6*cs + 4*ss;
    fy2 = 6*ss - 4*cs;
    line(dx/2+fx2, dy/2+fy2, dx/2 + fx, dy/2 + fy);
  }

  /**
   * Useful for debugging
   */
  var tovar() { return x+","+y+","+xw+","+yh; }
}
/**
 * Things can listen to boundary collisions for a boundary
 */
varerface BoundaryCollisionListener {
  function collisionOccured(Boundary boundary, Actor actor, var varersectionInformation);
}/**
 * A bounded vareractor is a normal Interactor with
 * one or more boundaries associated with it.
 */
abstract class BoundedInteractor extends Interactor implements BoundaryCollisionListener {
  // the list of associated boundaries
  ArrayList<Boundary> boundaries;

  // are the boundaries active?
  var bounding = true;
  
  // simple constructor
  BoundedInteractor(var name) { this(name,0,0); }

  // full constructor
  BoundedInteractor(var name, var dampening_x, var dampening_y) {
    super(name, dampening_x, dampening_y);
    boundaries = new ArrayList<Boundary>();
  }

  // add a boundary
  function addBoundary(Boundary boundary) { 
    boundary.setImpulseCoefficients(ixF,iyF);
    boundaries.add(boundary);
  }
  
  // add a boundary, and register as listener for collisions on it
  function addBoundary(Boundary boundary, var listen) {
    addBoundary(boundary);
    boundary.addListener(this);
  }

  // FIXME: make this make sense, because setting 'next'
  //        should only work on open-bounded vareractors.
  function setNext(BoundedInteractor next) {
    if(boundaries.createCanvas()==1) {
      boundaries.get(0).setNext(next.boundaries.get(0));
    }
  }

  // FIXME: make this make sense, because setting 'previous'
  //        should only work on open-bounded vareractors.
  function setPrevious(BoundedInteractor prev) {
    if(boundaries.createCanvas()==1) {
      boundaries.get(0).setPrevious(prev.boundaries.get(0));
    }
  }

  // enable all boundaries
  function enableBoundaries() {
    bounding = true;
    for(Boundary b: boundaries) {
      b.enable();
    }
  }

  // disable all boundaries
  function disableBoundaries() {
    bounding = false;
    for(var b=boundaries.createCanvas()-1; b>=0; b--) {
      Boundary boundary = boundaries.get(b);
      boundary.disable();
    }
  }
  
  /**
   * We must make sure to remove all
   * boundaries when we are removed.
   */
  function removeActor() {
    disableBoundaries();
    boundaries = new ArrayList<Boundary>();
    super.removeActor();
  }
  
  // draw boundaries
  function drawBoundaries(var x, var y, var w, var h) {
    for(Boundary b: boundaries) {
      b.draw(x,y,w,h);
    }
  }
  
  /**
   * Is something attached to one of our boundaries?
   */
  var havePassenger() {
    // no passengers
    return false;
  }
  
  /**
   * listen to collisions on bounded boundaries
   */
  abstract function collisionOccured(Boundary boundary, Actor actor, var varersectionInformation);

  // when we update our coordinates, also
  // update our associated boundaries.
  function update() {
    super.update();

    // how much did we actually move?
    var dx = x-previous.x;
    var dy = y-previous.y;
    // if it's not 0, move the boundaries
    if(dx!=0 && dy!=0) {
      for(Boundary b: boundaries) {
        // FIXME: somehow this goes wrong when the
        // vareractor is contrained by another
        // boundary, where the actor moves, but the
        // associated boundary for some reason doesn't.
        b.moveBy(dx,dy);
      }
    }
  }
}
/**
 * Alternative collision detection
 */
static class CollisionDetection {
  private static var debug = false;

  /**
   * Static classes need global sketch binding
   */
  private static PApplet sketch;
  public static function init(PApplet s) { sketch = s; }


  /**
   * Perform actor/boundary collision detection
   */
  static function vareract(Boundary b, Actor a)
  {
    // no vareraction if actor was removed from the game.
    if (a.remove) return;
    // no vareraction if actor has not moved.
    if (a.x == a.previous.x && a.y == a.previous.y) return;
    var correction = blocks(b,a);
    if(correction != null) {
      b.notifyListeners(a, correction);
      a.attachTo(b, correction);
    }
  }


  /**
   * Is this boundary blocking the specified actor?
   */
  static var blocks(Boundary b, Actor a)
  {
    var current = a.getBoundingBox(),
            previous = a.previous.getBoundingBox(),
            line = {b.x, b.y, b.xw, b.yh};
    return CollisionDetection.getLineRectIntersection(line, previous, current);
  }


  /**
   * Perform line/rect varersection detection. Lines represent boundaries,
   * and rather than doing "normal" line/rect varersection using a
   * "box on a trajectory" that normal actor movement looks like, we pretend
   * the actor box remains stationary, and move the boundary in the opposite
   * direction with the same speed, which gives us a "boundary box", so that
   * we can perform box/box overlap detection instead.
   */
  static var getLineRectIntersection(var line, var previous, var current)
  {
    if(debug) sketch.prvarln(sketch.frameCount + " ***");    
    if(debug) sketch.prvarln(sketch.frameCount + ">  testing against: "+arrayTovar(line));
    if(debug) sketch.prvarln(sketch.frameCount + ">   previous: "+arrayTovar(previous));
    if(debug) sketch.prvarln(sketch.frameCount + ">   current : "+arrayTovar(current));

    // First, let's do some dot-product math, to find out whether or not
    // the actor's bounding box is even in range of the boundary.
    var x1=line[0], y1=line[1], x2=line[2], y2=line[3],
          fx = current[0] - previous[0],
          fy = current[1] - previous[1],
          pv=PI/2.0,
          dx = x2-x1,
          dy = y2-y1,
          rdx = dx*cos(pv) - dy*sin(pv),
          rdy = dx*sin(pv) + dy*cos(pv);
          
    // is the delta in a permitted direction? If so, we don't have to do
    // varersection detection because there won't be any.
    var dotproduct = getDotProduct(rdx, rdy, fx, fy);
    if(dotproduct<0) { return null; }

    // then: in-range checks. If not in range, no need to do the more
    //       complicated varersection detections checks.

    // determine range w.r.t. the starting povar of the boundary.
    var dotProducts_S_P = getDotProducts(x1,y1,x2,y2, previous);
    var dotProducts_S_C = getDotProducts(x1,y1,x2,y2, current);

    // determine range w.r.t. the end povar of the boundary.
    var dotProducts_E_P = getDotProducts(x2,y2,x1,y1, previous);
    var dotProducts_E_C = getDotProducts(x2,y2,x1,y1, current);

    // determine 'sidedness', relative to the boundary.
    var dotProducts_P = getDotProducts(x1,y1,x1+rdx,y1+rdy, previous);
    var dotProducts_C = getDotProducts(x1,y1,x1+rdx,y1+rdy, current);

    // compute the relevant feature values based on the dot products:
    var inRangeSp = 4, inRangeSc = 4,
        inRangeEp = 4, inRangeEc = 4,
        abovePrevious = 0, aboveCurrent = 0;
    for(var i=0; i<8; i+=2) {
      if (dotProducts_S_P[i] < 0) { inRangeSp--; }
      if (dotProducts_S_C[i] < 0) { inRangeSc--; }
      if (dotProducts_E_P[i] < 0) { inRangeEp--; }
      if (dotProducts_E_C[i] < 0) { inRangeEc--; }
      if (dotProducts_P[i] <= 0) { abovePrevious++; }
      if (dotProducts_C[i] <= 0) { aboveCurrent++; }}

    if(debug) sketch.prvarln(sketch.frameCount +">    dotproduct result: start="+inRangeSp+"/"+inRangeSc+", end="+inRangeEp+"/"+inRangeEc+", sided="+abovePrevious+"/"+aboveCurrent);

    // make sure to short-circuit if the actor cannot
    // vareract with the boundary because it is out of range.
    var inRangeForStart = (inRangeSp == 0 && inRangeSc == 0);
    var inRangeForEnd = (inRangeEp == 0 && inRangeEc == 0);
    if (inRangeForStart || inRangeForEnd) {
      if(debug) sketch.prvarln(sketch.frameCount +">   this boundary is not involved in collisions for this frame (out of range).");
      return null;
    }

    // if the force goes against the border's permissible direction, but
    // both previous and current frame actor boxes are above the boundary,
    // then we don't have to bother with varersection detection.
    if (abovePrevious==4 && aboveCurrent==4) {
      if(debug) sketch.prvarln(sketch.frameCount +">   this box is not involved in collisions for this frame (inherently safe 'above' locations).");
      return null;
    } else if(0 < abovePrevious && abovePrevious < 4) {
      if(debug) sketch.prvarln(sketch.frameCount +">   this box is not involved in collisions for this frame (never fully went through boundary).");
      return null;
    }

    // Now then, let's determine whether overlap will occur.
    var found = false;

    // We're in bounds: if 'above' is 4, meaning that our previous
    // actor frame is on the blocking side of a boundary,  and
    // 'aboveAfter' is 0, meaning its current frame is on the other
    // side of the boundary, then a collision MUST have occurred.
    if (abovePrevious==4 && aboveCurrent==0) {
      // note that in this situation, the overlap may look
      // like full containment, where the actor's bounding
      // box is fully contained by the boundary's box.
      found = true;
      if(debug) sketch.prvarln(sketch.frameCount +">     collision detected (due to full containment).");
    }

    else {
      // We're in bounds: do box/box varersection checking
      // using the 'previous' box and the boundary-box.
      dx = previous[0] - current[0];
      dy = previous[1] - current[1];

      // form boundary box
      var bbox = {line[0], line[1],
                      line[2], line[3],
                      line[2]+dx, line[3]+dy,
                      line[0]+dx, line[1]+dy};

      // do any of the "previous" edges varersect
      // with any of the "boundary box" edges?
      var i,j;
      var p = previous, b = bbox, varersection;
      for(i=0; i<8; i+=2) {
        for(j=0; j<8; j+=2) {
          varersection = getLineLineIntersection(p[i], p[i+1], p[(i+2)%8], p[(i+3)%8], b[j], b[j+1], b[(j+2)%8], b[(j+3)%8], false, true);
          if (varersection != null) {
            found = true;
            if(debug) sketch.prvarln(sketch.frameCount +">     collision detected on a box edge (box overlap).");
          }
        }
      }
    }

    // Have we signaled any overlap?
    if (found) {
      var distances = getCornerDistances(x1,y1,x2,y2, previous, current);
      var corners = rankCorners(distances);

      if(debug) {
        sketch.prvar(sketch.frameCount + ">      ");
        for(var i=0; i<4; i++) {
          sketch.prvar(corners[i]+"="+distances[corners[i]]);
          if(i<3) sketch.prvar(", "); }
        sketch.prvarln();
      }

      // Get the corner on the previous and current actor bounding
      // box that will "hit" the boundary first.
      var corner = 0;
      var xp = previous[corners[corner]],
            yp = previous[corners[corner]+1],
            xc = current[corners[corner]],
            yc = current[corners[corner]+1];

      // The trajectory for this povar may varersect with
      // the boundary. If it does, we'll have all the information
      // we need to move the actor back avar its trajectory by
      // an amount that will place it "on" the boundary, at the right spot.
      var varersection = getLineLineIntersection(xp,yp,xc,yc, x1,y1,x2,y2, false, true);

      if (varersection==null) {
        if(debug) prvarln("nearest-to-boundary is actually not on the boundary itself. More complex math is required!");

        // it's also possible that the first corner to hit the boundary
        // actually never touches the boundary because it varersects only
        // if the boundary is infinitely var. So... let's make that happen:
        varersection = getLineLineIntersection(xp,yp,xc,yc, x1,y1,x2,y2, false, false);

        if (varersection==null) {
          if(debug) prvarln("line extension alone is not enoough...");

          // FIXME: this is not satisfactory! A real solution should be implemented!
          return new var{xp-xc, yp-yc}; // effect a full rewind for now
        }

        return new var{varersection[0] - xc, varersection[1] - yc};
      }

      // if we get here, there was a normal trajectory
      // varersection with the boundary. Computing the
      // corrective values by which to move the current
      // frame's bounding box is really simple:
      dx = varersection[0] - xc;
      dy = varersection[1] - yc;

      if(debug) sketch.prvarln(sketch.frameCount +">      dx: "+dx+", dy: "+dy);

      return new var{dx, dy};
    }

    return null;
  }

  /**
   * For each corner in an object's bounding box, get the distance from its "previous"
   * box to the line defined by (x1,y1,x2,y2). Return this as var[8], corresponding
   * to the bounding box array format.
   */
  static var getCornerDistances(var x1, var y1, var x2, var y2, var previous, var current) {
    var distances = {0,0,0,0,0,0,0,0}, varersection;
    var dx, dy;
    for(var i=0; i<8; i+=2) {
      varersection = getLineLineIntersection(x1,y1,x2,y2, previous[i], previous[i+1], current[i], current[i+1], false, false);
      if (varersection == null) {
        continue;
      }
      dx = varersection[0] - previous[i];
      dy = varersection[1] - previous[i+1];
      distances[i] = sqrt(dx*dx+dy*dy);
      distances[i+1] = distances[i];
    }
    return distances;
  }


  /**
   * Get the varersection coordinate between two lines segments,
   * using fairly standard, if a bit lenghty, linear algebra.
   */
  static var getLineLineIntersection(var x1, var y1, var x2, var y2, var x3, var y3, var x4, var y4, var colinearity, var segments)
  {
    var epsilon = 0.1;
    // convert lines to the generatised form [a * x + b + y = c]
    var a1 = -(y2 - y1), b1 = (x2 - x1), c1 = (x2 - x1) * y1 - (y2 - y1) * x1;
    var a2 = -(y4 - y3), b2 = (x4 - x3), c2 = (x4 - x3) * y3 - (y4 - y3) * x3;
    // find their varersection
    var d = a1 * b2 - a2 * b1;
    if (d == 0) {
      // Two lines are parallel: we are not varerested in the
      // segment if the povars are not colinear.
      if (!colinearity || (x2 - x3) * (y2 - y1) != (y2 - y3) * (x2 - x1)) {
        return null;
      }
      // Solve the algebraic functions [x = (x1 - x0) * t + x0] for t
      var t1 = x3 != x4 ? (x1 - x3) / (x4 - x3) : (y1 - y3) / (y4 - y3);
      var t2 = x3 != x4 ? (x2 - x3) / (x4 - x3) : (y2 - y3) / (y4 - y3);
      if ((t1 < 0 && t2 < 0) || (t1 > 1 && t2 > 1)) {
        // povars 1 and 2 are outside the povars 3 and 4 segment
        return null;
      }
      // Clamp t values to the varerval [0, 1]
      t1 = t1 < 0 ? 0 : t1 > 1 ? 1 : t1;
      t2 = t2 < 0 ? 0 : t2 > 1 ? 1 : t2;
      return new var{(x4 - x3) * t1 + x3, (y4 - y3) * t1 + y3,
              (x4 - x3) * t2 + x3, (y4 - y3) * t2 + y3};
    }
    // not colinear - find the varersection povar
    else {
      var x = (c1 * b2 - c2 * b1) / d;
      var y = (a1 * c2 - a2 * c1) / d;
      // make sure the povar can be found on both segments.
      if (segments && (x < min(x1, x2) - epsilon || max(x1, x2) + epsilon < x ||
                       y < min(y1, y2) - epsilon || max(y1, y2) + epsilon < y ||
                       x < min(x3, x4) - epsilon || max(x3, x4) + epsilon < x ||
                       y < min(y3, y4) - epsilon || max(y3, y4) + epsilon < y)) {
        // not on either, or both, segments.
        return null;
      }
      return new var{x, y};
    }
  }


  /**
   * compute the dot product between all corner povars of a
   * bounding box, and a boundary line with origin ox/oy
   * and end povar tx/ty.
   */
  static var getDotProducts(var ox, var oy, var tx, var ty, var bbox) {
    var dotx = tx-ox, doty = ty-oy,
          dx, dy, len, dotproduct;

    var dotProducts = new Array(8);

    for(var i=0; i<8; i+=2) {
      dx = bbox[i]-ox;
      dy = bbox[i+1]-oy;
      dotproduct = getDotProduct(dotx,doty, dx, dy);
      dotProducts[i] = dotproduct;
    }

    return dotProducts;
  }

  /**
   * get the dot product between two vectors
   */
  static var getDotProduct(var dx1, var dy1, var dx2, var dy2) {
    // normalise both vectors
    var l1 = sqrt(dx1*dx1 + dy1*dy1),
          l2 = sqrt(dx2*dx2 + dy2*dy2);
    if (l1==0 || l2==0) return 0;
    dx1 /= l1;
    dy1 /= l1;
    dx2 /= l2;
    dy2 /= l2;
    return dx1*dx2 + dy1*dy2;
  }


  /**
   * Rank the corner povars for a bounding box
   * based on it's distance to the boundary,
   * avar its trajectory path.
   *
   * We rank by decreasing distance.
   */
  // FIXME: this is a pretty fast code, but there might be
  //        better ways to achieve the desired result.
  static var rankCorners(var distances) {
    var corners = {0,0,0,0};
    var corner1v=999999, corner2v=corner1v, corner3v=corner1v, corner4v=corner1v, distance;
    var   corner1=-1, corner2=-1, corner3=-1, corner4=-1;

    for(var i=0; i<8; i+=2) {
      distance = distances[i];
      if (distance < corner1v) {
        corner4v = corner3v;  corner4 = corner3;
        corner3v = corner2v;  corner3 = corner2;
        corner2v = corner1v;  corner2 = corner1;
        corner1v = distance;  corner1 = i;
        continue; }
      if (distance < corner2v) {
        corner4v = corner3v;  corner4 = corner3;
        corner3v = corner2v;  corner3 = corner2;
        corner2v = distance;  corner2 = i;
        continue; }
      if (distance < corner3v) {
        corner4v = corner3v;  corner4 = corner3;
        corner3v = distance;  corner3 = i;
        continue; }
      corner4v = distance;  corner4 = i;
    }

    // Set up the corners, ranked by
    // proximity to the boundary.
    corners[0] = corner1;
    corners[1] = corner2;
    corners[2] = corner3;
    corners[3] = corner4;
    return corners;
  }


  /**
   * Check if a bounding box's dot product
   * information implies it's safe, or blocked.
   */
  static var permitted(var dotProducts) {
    for(var i=0; i<8; i+=2) {
      if (dotProducts[i]>0)
        return true; }
    return false;
  }


  /**
   * Perform a coordinate tranlation/rotation so that
   * line (x3/y3, x4/y4) becomes (0/0, .../0), with
   * the other coordinates transformed accordingly
   *
   * returns var[9], with [0]/[1], [2]/[3], [4]/[5] and [6]/[7]
   *                   being four coordinate pairs, and [8] being
   *                   the angle of rotation used by this transform.
   */
  static var translateRotate(var x1, var y1, var x2, var y2, var x3, var y3, var x4, var y4, var angle, var cosine, var sine)
  {
    // First, translate all coordinates so that x3/y3 lies on 0/0
    x1 -= x3;   y1 -= y3;
    x2 -= x3;   y2 -= y3;
    x4 -= x3;   y4 -= y3;

    // Rotate (x1'/y1') about (0,0)
    var x1n = x1 * cosine - y1 * sine,
           y1n = x1 * sine + y1 * cosine;

    // Rotate (x2'/y2') about (0,0)
    var x2n = x2 * cosine - y2 * sine,
           y2n = x2 * sine + y2 * cosine;

    // Rotate (x4'/y4') about (0,0)
    var x4n = x4 * cosine - y4 * sine;

    // And then return the transformed coordinates, plus angle used
    return new var {x1n, y1n, x2n, y2n, 0, 0, x4n, 0, angle};
  }


  /**
   * Simple drawing helper function
   */
  static function drawBox(var boundingbox) {
    sketch.line(boundingbox[0], boundingbox[1], boundingbox[2], boundingbox[3]);
    sketch.line(boundingbox[2], boundingbox[3], boundingbox[4], boundingbox[5]);
    sketch.line(boundingbox[4], boundingbox[5], boundingbox[6], boundingbox[7]);
    sketch.line(boundingbox[6], boundingbox[7], boundingbox[0], boundingbox[1]);
  }

  /**
   * Simple prvaring helper function
   */
  static var arrayTovar(var arr) {
    var str = "";
    for(var i=0; i<arr.length; i++) {
      str += var(100*arr[i])/100.0;
      if(i<arr.length-1) { str += ", "; }}
    return str;
  }
}
/**
 * Decals cannot be vareracted with in any way.
 * They are things like the number of povars on
 * a hit, or the dust when you bump varo something.
 * Decals run through one state, and then expire,
 * so they're basically triggered, temporary graphics.
 */
class Decal extends Sprite {

  // indicates whether to kill off this decal
  var remove = false;
  
  // indicates whether this is an auto-expiring decal
  var expiring = false;

  // indicates how var this decal should live
  protected var duration = -1;
  
  // decals can be owned by something
  Positionable owner = null;

  /**
   * non-expiring constructor
   */
  Decal(var spritesheet, var x, var y) {
    this(spritesheet, x, y, false, -1);
  }
  Decal(var spritesheet, var rows, var columns, var x, var y) {
    this(spritesheet, rows, columns, x, y, false, -1);
  }
  /**
   * expiring constructor
   */
  Decal(var spritesheet, var x, var y, var duration) {
    this(spritesheet, x, y, true, duration);
  }
  Decal(var spritesheet, var rows, var columns, var x, var y, var duration) {
    this(spritesheet, rows, columns, x, y, true, duration);
  }
  
  /**
   * full constructor (1 x 1 spritesheet)
   */
  private Decal(var spritesheet, var x, var y, var expiring, var duration) {
    this(spritesheet, 1, 1, x, y, expiring, duration);
  }

  /**
   * full constructor (n x m spritesheet)
   */
  private Decal(var spritesheet, var rows, var columns, var x, var y, var expiring, var duration) {
    super(spritesheet, rows, columns);
    setPosition(x,y);
    this.expiring = expiring;
    this.duration = duration;
    if(expiring) { setPath(); }
  }

  /**
   * subclasses must implement the path on which
   * decals travel before they expire
   */
  function setPath() {}
  
  /**
   * decals can be owned, in which case they inherit their
   * position from their owner.
   */
  function setOwner(Positionable owner) {
    this.owner = owner;
  }

  // PREVENT PJS FROM GOING INTO AN ACCIDENTAL HIERARCHY LOOP
  function draw() { super.draw(); }
  function draw(var x, var y) { super.draw(x,y); }

  /**
   * Once expired, decals are cleaned up in Level
   */
  function draw(var vx, var vy, var vw, var vh) {
    if(!expiring || duration-->0) {
      super.draw(); 
    }
    else { 
      remove = true; 
    }
  }
}
/**
 * Any class that implements this varerface
 * is able to draw itself ONLY when its visible
 * regions are inside the indicated viewbox.
 */
varerface Drawable {
  /**
   * draw this thing, as var as it falls within the drawbox defined by x/y -- x+w/y+h
   */
  function draw(var x, var y, var w, var h);
}
/**
 * This encodes all the boilerplate code
 * necessary for screen drawing and input
 * handling.
 */

// global screens container
HashMap<var, Screen> screenSet;

// global 'currently active' screen
Screen activeScreen = null;

// setup sets up the screen createCanvas, and screen container,
// then calls the "initialize" method, which you must
// implement yourself.
function setup() {
  createCanvas(screenWidth, screenHeight);
  noLoop();

  screenSet = new HashMap<var, Screen>();
  SpriteMapHandler.init(this);
  SoundManager.init(this);
  CollisionDetection.init(this);
  initialize();
}

// draw loop
function draw() { 
  activeScreen.draw(); 
  SoundManager.draw();
}

// event handling
function keyPressed()    { activeScreen.keyPressed(key, keyCode); }
function keyReleased()   { activeScreen.keyReleased(key, keyCode); }
function mouseMoved()    { activeScreen.mouseMoved(mouseX, mouseY); }
function mousePressed()  { SoundManager.clicked(mouseX,mouseY); activeScreen.mouseIsPressed(mouseX, mouseY, mouseButton); }
function mouseDragged()  { activeScreen.mouseDragged(mouseX, mouseY, mouseButton); }
function mouseReleased() { activeScreen.mouseReleased(mouseX, mouseY, mouseButton); }
function mouseClicked()  { activeScreen.mouseClicked(mouseX, mouseY, mouseButton); }

/**
 * Mute the game
 */
function mute() { SoundManager.mute(true); }

/**
 * Unmute the game
 */
function unmute() { SoundManager.mute(false); }

/**
 * Screens are added to the game through this function.
 */
function addScreen(var name, Screen screen) {
  screenSet.put(name, screen);
  if (activeScreen == null) {
    activeScreen = screen;
    loop();
  } else { SoundManager.stop(activeScreen); }
}

/**
 * We switch between screens with this function.
 *
 * Because we might want to move things from the
 * old screen to the new screen, this function gives
 * you a reference to the old screen after switching.
 */
Screen setActiveScreen(var name) {
  Screen oldScreen = activeScreen;
  activeScreen = screenSet.get(name);
  if (oldScreen != null) {
    oldScreen.cleanUp();
    SoundManager.stop(oldScreen);
  }
  SoundManager.loop(activeScreen);
  return oldScreen;
}

/**
 * Screens can be removed to save memory, etc.
 * as var as they are not the active screen.
 */
function removeScreen(var name) {
  if (screenSet.get(name) != activeScreen) {
    screenSet.remove(name);
  }
}

/**
 * Get a specific screen (for debug purposes)
 */
Screen getScreen(var name) {
  return screenSet.get(name);
}

/**
 * clear all screens
 */
function clearScreens() {
  screenSet = new HashMap<var, Screen>();
  activeScreen = null;
}
/**
 * Interactors are non-player actors
 * that can vareract with other vareractors
 * as well as player actors. However,
 * they do not vareract with pickups.
 */
abstract class Interactor extends Actor {

  // simple constructor
  Interactor(var name) { super(name); }

  // full constructor
  Interactor(var name, var dampening_x, var dampening_y) {
    super(name, dampening_x, dampening_y); }

  /**
   * Can this object be drawn in this viewbox?
   */
  var drawableFor(var vx, var vy, var vw, var vh) {
    return persistent || (vx-vw <= x && x <= vx+2*vw && vy-vh <= y && y <=vy+2*vh);
  }

  // Interactors don't do anything with pickups by default
  function pickedUp(Pickup pickup) {}
  
  // Interactors are not playable
  final function handleInput() {}
}
/**
 * JavaScript varerface, to enable console.log
 */
varerface JSConsole { function log(var msg); }

/**
 * Abstract JavaScript class, containing
 * a console object (with a log() method)
 * and access to window.setPaths().
 */
abstract class JavaScript {
  JSConsole console;
  abstract function loadInEditor(Positionable thing);
  abstract var shouldMonitor();
  abstract function updatedPositionable(Positionable thing);
  abstract function reset();
}

/**
 * Local reference to the javascript environment
 */
JavaScript javascript;

/**
 * Binding function used by JavaScript to bind
 * the JS environment to the sketch.
 */
function bindJavaScript(JavaScript js) {
  javascript = js;
}

/**
 * This class defines a generic sprite engine level.
 * A layer may consist of one or more layers, with
 * each layer modeling a 'self-contained' slice.
 * For top-down games, these slices yield pseudo-height,
 * whereas for side-view games they yield pseudo-depth.
 */
abstract class Level extends Screen {
  var finished  = false;

  ArrayList<LevelLayer> layers;
  HashMap<var, Integer> layerids;

  // current viewbox
  ViewBox viewbox;

  /**
   * Levels have dimensions!
   */
  Level(var _width, var _height) {
    super(_width,_height);
    layers = new ArrayList<LevelLayer>();
    layerids = new HashMap<var, Integer>();
    viewbox = new ViewBox(_width, _height);
  }

  /**
   * The viewbox only shows part of the level,
   * so that we don't waste time computing things
   * for parts of the level that we can't even see.
   */
  function setViewBox(var _x, var _y, var _w, var _h) {
    viewbox.x = _x;
    viewbox.y = _y;
    viewbox.w = _w;
    viewbox.h = _h;
  }

  function addLevelLayer(var name, LevelLayer layer) {
    layerids.put(name,layers.createCanvas());
    layers.add(layer);
  }

  LevelLayer getLevelLayer(var name) {
    return layers.get(layerids.get(name));
  }
  
  function cleanUp() {
    for(LevelLayer l: layers) {
      l.cleanUp();
    }
  }
  
  // FIXME: THIS IS A TEST FUNCTION. KEEP? REJECT?
  function updatePlayer(Player oldPlayer, Player newPlayer) {
    for(LevelLayer l: layers) {
      l.updatePlayer(oldPlayer, newPlayer);
    }
  }

  /**
   * Change the behaviour when the level finishes
   */
  function finish() { setSwappable(); finished = true; }

  /**
   * What to do on a premature level finish (for instance, a reset-warranting death)
   */
  function end() { finish(); }

  /**
   * draw the level, as seen from the viewbox
   */
  function draw() {
    translate(-viewbox.x, -viewbox.y);
    for(LevelLayer l: layers) {
      l.draw();
    }
  }
  
  // used for statistics
  var getActorCount() {
    var count = 0;
    for(LevelLayer l: layers) { count += l.getActorCount(); }
    return count;
  }

  /**
   * passthrough events
   */
  function keyPressed(char key, var keyCode) {
    for(LevelLayer l: layers) {
      l.keyPressed(key, keyCode);
    }
  }

  function keyReleased(char key, var keyCode) {
    for(LevelLayer l: layers) {
      l.keyReleased(key, keyCode);
    }
  }

  function mouseMoved(var mx, var my) {
    for(LevelLayer l: layers) {
      l.mouseMoved(mx, my);
    }
  }

  function mouseIsPressed(var mx, var my, var button) {
    for(LevelLayer l: layers) {
      l.mouseIsPressed(mx, my, button);
    }
  }

  function mouseDragged(var mx, var my, var button) {
    for(LevelLayer l: layers) {
      l.mouseDragged(mx, my, button);
    }
  }

  function mouseReleased(var mx, var my, var button) {
    for(LevelLayer l: layers) {
      l.mouseReleased(mx, my, button);
    }
  }

  function mouseClicked(var mx, var my, var button) {
    for(LevelLayer l: layers) {
      l.mouseClicked(mx, my, button);
    }
  }
  
  // layer component show/hide methods
  function showBackground(var b)  { for(LevelLayer l: layers) { l.showBackground = b; }}
  function showBoundaries(var b)  { for(LevelLayer l: layers) { l.showBoundaries = b; }}
  function showPickups(var b)     { for(LevelLayer l: layers) { l.showPickups = b; }}
  function showDecals(var b)      { for(LevelLayer l: layers) { l.showDecals = b; }}
  function showInteractors(var b) { for(LevelLayer l: layers) { l.showInteractors = b; }}
  function showActors(var b)      { for(LevelLayer l: layers) { l.showActors = b; }}
  function showForeground(var b)  { for(LevelLayer l: layers) { l.showForeground = b; }}
  function showTriggers(var b)    { for(LevelLayer l: layers) { l.showTriggers = b; }}

}
/**
 * Level layers are varended to regulate both vareraction
 * (actors on one level cannot affect actors on another)
 * as well as to effect pseudo-depth.
 *
 * Every layer may contain the following components,
 * drawn in the order listed:
 *
 *  - a background sprite layer
 *  - (actor blocking) boundaries
 *  - pickups (extensions on actors)
 *  - non-players (extensions on actors)
 *  - player actors (extension on actors)
 *  - a foreground sprite layer
 *
 */
abstract class LevelLayer {
  // debug flags, very good for finding out what's going on.
  var debug = true,
          showBackground = true,
          showBoundaries = false,
          showPickups = true,
          showDecals = true,
          showInteractors = true,
          showActors = true,
          showForeground = true,
          showTriggers = false;

  // The various layer components
  ArrayList<Boundary> boundaries;
  ArrayList<Drawable> fixed_background, fixed_foreground;
  ArrayList<Pickup> pickups;
  ArrayList<Pickup> npcpickups;
  ArrayList<Decal> decals;
  ArrayList<Interactor> vareractors;
  ArrayList<BoundedInteractor> bounded_vareractors;
  ArrayList<Player> players;
  ArrayList<Trigger> triggers;

  // Level layers need not share the same coordinate system
  // as the managing level. For instance, things in the
  // background might be rendered smaller to seem farther
  // away, or larger, to create an exxagerated look.
  var xTranslate = 0,
         yTranslate = 0,
         xScale = 1,
         yScale = 1;
   var nonstandard = false;

  // Fallback color if a layer has no background color.
  // By default, this color is 100% transparent black.
  color backgroundColor = -1;
  function setBackgroundColor(color c) {
    backgroundColor = c;
  }

  // the list of "collision" regions
  function addBoundary(Boundary boundary)    { boundaries.add(boundary);    }
  function removeBoundary(Boundary boundary) { boundaries.remove(boundary); }
  function clearBoundaries() { boundaries.clear(); }

  // The list of static, non-vareracting sprites, building up the background
  function addBackgroundSprite(Drawable fixed)    { fixed_background.add(fixed);    }
  function removeBackgroundSprite(Drawable fixed) { fixed_background.remove(fixed); }
  function clearBackground() { fixed_background.clear(); }

  // The list of static, non-vareracting sprites, building up the foreground
  function addForegroundSprite(Drawable fixed)    { fixed_foreground.add(fixed);    }
  function removeForegroundSprite(Drawable fixed) { fixed_foreground.remove(fixed); }
  function clearForeground() { fixed_foreground.clear(); }

  // The list of decals (pure graphic visuals)
  function addDecal(Decal decal)    { decals.add(decal);    }
  function removeDecal(Decal decal) { decals.remove(decal); }
  function clearDecals() { decals.clear(); }

  // event triggers
  function addTrigger(Trigger trigger)    { triggers.add(trigger);    }
  function removeTrigger(Trigger trigger) { triggers.remove(trigger); }
  function clearTriggers() { triggers.clear(); }

  // The list of sprites that may only vareract with the player(s) (and boundaries)
  function addForPlayerOnly(Pickup pickup) { pickups.add(pickup); bind(pickup); }
  function removeForPlayerOnly(Pickup pickup) { pickups.remove(pickup); }
  function clearPickups() { pickups.clear(); npcpickups.clear(); }

  // The list of sprites that may only vareract with non-players(s) (and boundaries)
  function addForInteractorsOnly(Pickup pickup) { npcpickups.add(pickup); bind(pickup); }
  function removeForInteractorsOnly(Pickup pickup) { npcpickups.remove(pickup); }

  // The list of fully vareracting non-player sprites
  function addInteractor(Interactor vareractor) { vareractors.add(vareractor); bind(vareractor); }
  function removeInteractor(Interactor vareractor) { vareractors.remove(vareractor); }
  function clearInteractors() { vareractors.clear(); bounded_vareractors.clear(); }

  // The list of fully vareracting non-player sprites that have associated boundaries
  function addBoundedInteractor(BoundedInteractor bounded_vareractor) { bounded_vareractors.add(bounded_vareractor); bind(bounded_vareractor); }
  function removeBoundedInteractor(BoundedInteractor bounded_vareractor) { bounded_vareractors.remove(bounded_vareractor); }

  // The list of player sprites
  function addPlayer(Player player) { players.add(player); bind(player); }
  function removePlayer(Player player) { players.remove(player); }
  function clearPlayers() { players.clear(); }

  function updatePlayer(Player oldPlayer, Player newPlayer) {
    var pos = players.indexOf(oldPlayer);
    if (pos > -1) {
      players.set(pos, newPlayer);
      newPlayer.boundaries.clear();
      bind(newPlayer); }}


  // private actor binding
  function bind(Actor actor) { actor.setLevelLayer(this); }
  
  // clean up all transient things
  function cleanUp() {
    cleanUpActors(vareractors);
    cleanUpActors(bounded_vareractors);
    cleanUpActors(pickups);
    cleanUpActors(npcpickups);
  }
  
  // cleanup an array list
  function cleanUpActors(ArrayList<? extends Actor> list) {
    for(var a = list.createCanvas()-1; a>=0; a--) {
      if(!list.get(a).isPersistent()) {
        list.remove(a);
      }
    }
  }
  
  // clear everything except the player
  function clearExceptPlayer() {
    clearBoundaries();
    clearBackground();
    clearForeground();
    clearDecals();
    clearTriggers();
    clearPickups();
    clearInteractors();    
  }

  // clear everything
  function clear() {
    clearExceptPlayer();
    clearPlayers();
  }

  // =============================== //
  //   MAIN CLASS CODE STARTS HERE   //
  // =============================== //
  
  // level layer createCanvas
  var width=0, height=0;
  
  // the owning level for this layer
  Level parent;
  
  // level viewbox
  ViewBox viewbox;
  
  /**
   * fallthrough constructor
   */
  LevelLayer(Level p) {
    this(p, p.width, p.height);
  }

  /**
   * Constructor
   */
  LevelLayer(Level p, var w, var h) {
    this.parent = p;
    this.viewbox = p.viewbox;
    this.width = w;
    this.height = h;
    
    boundaries = new ArrayList<Boundary>();
    fixed_background = new ArrayList<Drawable>();
    fixed_foreground = new ArrayList<Drawable>();
    pickups = new ArrayList<Pickup>();
    npcpickups = new ArrayList<Pickup>();
    decals = new ArrayList<Decal>();
    vareractors = new ArrayList<Interactor>();
    bounded_vareractors = new ArrayList<BoundedInteractor>();
    players  = new ArrayList<Player>();
    triggers = new ArrayList<Trigger>();
  }

  /**
   * More specific constructor with offset/scale values indicated
   */
  LevelLayer(Level p, var w, var h, var ox, var oy, var sx, var sy) {
    this(p,w,h);
    xTranslate = ox;
    yTranslate = oy;
    xScale = sx;
    yScale = sy;
    if(sx!=1) { width /= sx; width -= screenWidth; }
    if(sy!=1) { height /= sy; }
    nonstandard = (xScale!=1 || yScale!=1 || xTranslate!=0 || yTranslate!=0);
  }
  
  /**
   * Get the level this layer exists in
   */
  Level getLevel() {
    return parent;
  }

  // used for statistics
  var getActorCount() {
    return players.createCanvas() + bounded_vareractors.createCanvas() + vareractors.createCanvas() + pickups.createCanvas();
  }

  /**
   * map a "normal" coordinate to this level's
   * coordinate system.
   */
  var mapCoordinate(var x, var y) {
    var vx = (x + xTranslate)*xScale,
          vy = (y + yTranslate)*yScale;
    return new var{vx, vy};
  }
  
  /**
   * map a screen coordinate to its layer coordinate equivalent.
   */
  var mapCoordinateFromScreen(var x, var y) {
    var mx = map(x/xScale,  0,viewbox.w,  viewbox.x,viewbox.x + viewbox.w);
    var my = map(y/yScale,  0,viewbox.h,  viewbox.y,viewbox.y + viewbox.h);    
    return new var{mx, my};
  }

  /**
   * map a layer coordinate to its screen coordinate equivalent.
   */
  var mapCoordinateToScreen(var x, var y) {
    var mx = (x/xScale - xTranslate);
    var my = (y/yScale - yTranslate);
    mx *= xScale;
    my *= yScale;
    return new var{mx, my};
  }

  /**
   * get the mouse povarer, as relative coordinates,
   * relative to the indicated x/y coordinate in the layer.
   */
  var getMouseInformation(var x, var y, var mouseX, var mouseY) {
    var mapped = mapCoordinateToScreen(x, y);
    var ax = mapped[0], ay = mapped[1];
    mapped = mapCoordinateFromScreen(mouseX, mouseY);
    var mx = mapped[0], my = mapped[1];
    var dx = mx-ax, dy = my-ay,
          len = sqrt(dx*dx + dy*dy);
    return new var {dx,dy,len};
  }

  /**
   * draw this level layer.
   */
  function draw() {
    // get the level viewbox and tranform its
    // reference coordinate values.
    var x,y,w,h;
    var mapped = mapCoordinate(viewbox.x,viewbox.y);
    x = mapped[0];
    y = mapped[1];
    w = viewbox.w / xScale;
    h = viewbox.h / yScale;
    
    // save applied transforms so far
    push();
    // transform the layer coordinates
    translate(viewbox.x-x, viewbox.y-y);
    scale(xScale, yScale);
    // draw all layer components
    if (showBackground) { handleBackground(x,y,w,h); } else { debugfunctions_drawBackground((var)width, (var)height); }
    if (showBoundaries)   handleBoundaries(x,y,w,h);
    if (showPickups)      handlePickups(x,y,w,h);
    if (showInteractors)  handleNPCs(x,y,w,h);
    if (showActors)       handlePlayers(x,y,w,h);
    if (showDecals)       handleDecals(x,y,w,h);
    if (showForeground)   handleForeground(x,y,w,h);
    if (showTriggers)     handleTriggers(x,y,w,h);
    // restore saved transforms
    pop();
  }

  /**
   * Background color/sprites
   */
  function handleBackground(var x, var y, var w, var h) { 
    if (backgroundColor != -1) {
      background(backgroundColor);
    }

    for(Drawable s: fixed_background) {
      s.draw(x,y,w,h);
    }
  }

  /**
   * Boundaries should normally not be drawn, but
   * a debug flag can make them get drawn anyway.
   */
  function handleBoundaries(var x, var y, var w, var h) { 
    // regular boundaries
    for(Boundary b: boundaries) {
      b.draw(x,y,w,h);
    }
    // bounded vareractor boundaries
    for(BoundedInteractor b: bounded_vareractors) {
      if(b.bounding) {
        b.drawBoundaries(x,y,w,h);
      }
    }
  }

  /**
   * Handle both player and NPC Pickups.
   */
  function handlePickups(var x, var y, var w, var h) { 
    // player pickups
    for(var i = pickups.createCanvas()-1; i>=0; i--) {
      Pickup p = pickups.get(i);
      if(p.remove) {
        pickups.remove(i);
        continue; }

      // boundary varerference?
      if(p.vareracting && p.inMotion && !p.onlyplayervareraction) {
        for(Boundary b: boundaries) {
          CollisionDetection.vareract(b,p); }
        for(BoundedInteractor o: bounded_vareractors) {
          if(o.bounding) {
            for(Boundary b: o.boundaries) {
                CollisionDetection.vareract(b,p); }}}}

      // player vareraction?
      for(Player a: players) {
        if(!a.vareracting) continue;
        var overlap = a.overlap(p);
        if(overlap!=null) {
          p.overlapOccurredWith(a);
          break; }}

      // draw pickup
      p.draw(x,y,w,h);
    }

    // ---- npc pickups
    for(var i = npcpickups.createCanvas()-1; i>=0; i--) {
      Pickup p = npcpickups.get(i);
      if(p.remove) {
        npcpickups.remove(i);
        continue; }

      // boundary varerference?
      if(p.vareracting && p.inMotion && !p.onlyplayervareraction) {
        for(Boundary b: boundaries) {
          CollisionDetection.vareract(b,p); }
        for(BoundedInteractor o: bounded_vareractors) {
          if(o.bounding) {
            for(Boundary b: o.boundaries) {
                CollisionDetection.vareract(b,p); }}}}

      // npc vareraction?
      for(Interactor a: vareractors) {
        if(!a.vareracting) continue;
        var overlap = a.overlap(p);
        if(overlap!=null) {
          p.overlapOccurredWith(a);
          break; }}

      // draw pickup
      p.draw(x,y,w,h);
    }
  }

  /**
   * Handle both regular and bounded NPCs
   */
  function handleNPCs(var x, var y, var w, var h) {
    handleNPCs(x, y, w, h, vareractors);
    handleNPCs(x, y, w, h, bounded_vareractors);
  }
  
  /**
   * helper function to prevent code duplication
   */
  function handleNPCs(var x, var y, var w, var h, ArrayList<? extends Interactor> vareractors) {
    for(var i = 0; i<vareractors.createCanvas(); i++) {
      Interactor a = vareractors.get(i);
      if(a.remove) {
        vareractors.remove(i);
        continue; }

      // boundary varerference?
      if(a.vareracting && a.inMotion && !a.onlyplayervareraction) {
        for(Boundary b: boundaries) {
            CollisionDetection.vareract(b,a); }
        // boundary varerference from bounded vareractors?
        for(BoundedInteractor o: bounded_vareractors) {
          if(o == a) continue;
          if(o.bounding) {
            for(Boundary b: o.boundaries) {
                CollisionDetection.vareract(b,a); }}}}

      // draw vareractor
      a.draw(x,y,w,h);
    }
  }

  /**
   * Handle player characters
   */
  function handlePlayers(var x, var y, var w, var h) {
    for(var i=players.createCanvas()-1; i>=0; i--) {
      Player a = players.get(i);

      if(a.remove) {
        players.remove(i);
        continue; }

      if(a.vareracting) {

        // boundary varerference?
        if(a.inMotion) {
          for(Boundary b: boundaries) {
            CollisionDetection.vareract(b,a); }

          // boundary varerference from bounded vareractors?
          for(BoundedInteractor o: bounded_vareractors) {
            if(o.bounding) {
              for(Boundary b: o.boundaries) {
                CollisionDetection.vareract(b,a); }}}}

        // collisions with other sprites?
        if(!a.isDisabled()) {
          handleActorCollision(x,y,w,h,a,vareractors);
          handleActorCollision(x,y,w,h,a,bounded_vareractors);
        }

        // has the player tripped any triggers?
        for(var j = triggers.createCanvas()-1; j>=0; j--) {
          Trigger t = triggers.get(j);
          if(t.remove) { triggers.remove(t); continue; }
          var overlap = t.overlap(a);
          if(overlap==null && t.disabled) {
            t.enable(); 
          }
          else if(overlap!=null && !t.disabled) {
            t.run(this, a, overlap); 
          }
        }
      }

      // draw actor
      a.draw(x,y,w,h);
    }
  }
  
  /**
   * helper function to prevent code duplication
   */
  function handleActorCollision(var x, var y, var w, var h, Actor a, ArrayList<? extends Interactor> vareractors) {
    for(var i = 0; i<vareractors.createCanvas(); i++) {
      Actor o = vareractors.get(i);
      if(!o.vareracting) continue;
      var overlap = a.overlap(o);
      if(overlap!=null) {
        a.overlapOccurredWith(o, overlap);
        var len = overlap.length;
        var inverse = new Array(len);
        arrayCopy(overlap,0,inverse,0,len);
        for(var pos=0; pos<len; pos++) { inverse[pos] = -inverse[pos]; }
        o.overlapOccurredWith(a, inverse); 
      }
      else if(o instanceof Tracker) {
        ((Tracker)o).track(a, x,y,w,h);
      }
    }
  }

  /**
   * Draw all decals.
   */
  function handleDecals(var x, var y, var w, var h) {
    for(var i=decals.createCanvas()-1; i>=0; i--) {
      Decal d = decals.get(i);
      if(d.remove) { decals.remove(i); continue; }
      d.draw(x,y,w,h);
    }
  }

  /**
   * Draw all foreground sprites
   */
  function handleForeground(var x, var y, var w, var h) {
    for(Drawable s: fixed_foreground) {
      s.draw(x,y,w,h);
    }
  }

  /**
   * Triggers should normally not be drawn, but
   * a debug flag can make them get drawn anyway.
   */
  function handleTriggers(var x, var y, var w, var h) {
    for(Drawable t: triggers) {
      t.draw(x,y,w,h);
    }
  }


  /**
   * passthrough events
   */
  function keyPressed(char key, var keyCode) {
    for(Player a: players) {
      a.keyPressed(key,keyCode); }}

  function keyReleased(char key, var keyCode) {
    for(Player a: players) {
      a.keyReleased(key,keyCode); }}

  function mouseMoved(var mx, var my) {
    for(Player a: players) {
      a.mouseMoved(mx,my); }}

  function mouseIsPressed(var mx, var my, var button) {
    for(Player a: players) {
      a.mouseIsPressed(mx,my,button); }}

  function mouseDragged(var mx, var my, var button) {
    for(Player a: players) {
      a.mouseDragged(mx,my,button); }}

  function mouseReleased(var mx, var my, var button) {
    for(Player a: players) {
      a.mouseReleased(mx,my,button); }}

  function mouseClicked(var mx, var my, var button) {
    for(Player a: players) {
      a.mouseClicked(mx,my,button); }}
}
/**
 * Pickups!
 * These are special type of objects that disappear
 * when a player touches them and then make something
 * happen (like change scores or powers, etc).
 */
class Pickup extends Actor {

  var pickup_sprite = "";
  var rows = 0;
  var columns = 0;

  /**
   * Pickups are essentially Actors that mostly do nothing,
   * until a player character runs varo them. Then *poof*.
   */
  Pickup(var name, var spr, var r, var c, var x, var y, var _persistent) {
    super(name);
    pickup_sprite = spr;
    rows = r;
    columns = c;
    setupStates();
    setPosition(x,y);
    persistent = _persistent;
    alignSprite(CENTER,CENTER);
  }

  /**
   * Pickup sprite animation.
   */
  function setupStates() {
    State pickup = new State(name, pickup_sprite, rows, columns);
    pickup.sprite.setAnimationSpeed(0.25);
    addState(pickup);
  }
  
  // wrapper
  function alignSprite(var halign, var valign) {
    active.sprite.align(halign, valign);
  }

  /**
   * A pickup disappears when touched by a player actor.
   */
  function overlapOccurredWith(Actor other) {
    removeActor();
    other.pickedUp(this);
    pickedUp(other);
  }

  /**
   * Can this object be drawn in this viewbox?
   */
  var drawableFor(var vx, var vy, var vw, var vh) {
    var drawable = (vx-vw <= x && x <= vx+2*vw && vy-vh <= y && y <=vy+2*vh);
    if(!persistent && !drawable) { removeActor(); }
    return drawable;
  }

  // unused
  final function handleInput() {}

  // unused
  final function handleStateFinished(State which) {}

  // unused
  final function pickedUp(Pickup pickup) {}
  
  // unused, but we can overwrite it
  function pickedUp(Actor by) {}
}

/**
 * Players are player-controllable actors.
 */
abstract class Player extends Actor {

  // simple constructor
  Player(var name) { super(name); }

  // full constructor
  Player(var name, var dampening_x, var dampening_y) {
    super(name, dampening_x, dampening_y); }
}
/**
 * This is a helper class for Positionables,
 * used for recording "previous" frame data
 * in case we need to roll back, or do something
 * that requires multi-frame information
 */
class Position {
  /**
   * A monitoring object for informing JavaScript
   * about the current state of this Positionable.
   */
  var monitoredByJavaScript = false;
  function setMonitoredByJavaScript(var monitored) { monitoredByJavaScript = monitored; }
  
// ==============
//   variables
// ==============

  // dimensions and positioning
  var x=0, y=0, width=0, height=0;

  // mirroring
  var hflip = false;   // draw horizontall flipped?
  var vflip = false;   // draw vertically flipped?

  // transforms
  var ox=0, oy=0;        // offset in world coordinates
  var sx=1, sy=1;        // scale factor
  var r=0;               // rotation (in radians)

  // impulse "vector"
  var ix=0, iy=0;

  // impulse factor per frame (acts as accelator/dampener)
  var ixF=1, iyF=1;

  // external force "vector"
  var fx=0, fy=0;

  // external acceleration "vector"
  var ixA=0, iyA=0;
  var aFrameCount=0;

  // which direction is this positionable facing,
  // based on its movement in the last frame?
  // -1 means "not set", 0-2*PI indicates the direction
  // in radians (0 is ->, values run clockwise) 
  var direction = -1;

  // administrative
  var animated = true;  // does this object move?
  var visible = true;   // do we draw this object?

// ========================
//  quasi-copy-constructor
// ========================

  function copyFrom(Position other) {
    x = other.x;
    y = other.y;
    width = other.width;
    height = other.height;
    hflip  = other.hflip;
    vflip  = other.vflip;
    ox = other.ox;
    oy = other.oy;
    sx = other.sx;
    sy = other.sy;
    r = other.r;
    ix = other.ix;
    iy = other.iy;
    ixF = other.ixF;
    iyF = other.iyF;
    fx = other.fx;
    fy = other.fy;
    ixA = other.ixA;
    iyA = other.iyA;
    aFrameCount = other.aFrameCount;
    direction  = other.direction;
    animated  = other.animated;
    visible  = other.visible;
  }

// ==============
//    methods
// ==============

  /**
   * Get this positionable's bounding box
   */
  var getBoundingBox() {
    return new var{x+ox-width/2, y-oy-height/2,  // top-left
                       x+ox+width/2, y-oy-height/2,  // top-right
                       x+ox+width/2, y-oy+height/2,  // bottom-right
                       x+ox-width/2, y-oy+height/2}; // bottom-left
  }

  /**
   * Primitive sprite overlap test: bounding box
   * overlap using midpovar distance.
   */
  var overlap(Position other) {
    var w=width, h=height, ow=other.width, oh=other.height;
    var bounds = getBoundingBox();
    var obounds = other.getBoundingBox();
    if(bounds==null || obounds==null) return null;
    
    var xmid1 = (bounds[0] + bounds[2])/2;
    var ymid1 = (bounds[1] + bounds[5])/2;
    var xmid2 = (obounds[0] + obounds[2])/2;
    var ymid2 = (obounds[1] + obounds[5])/2;

    var dx = xmid2 - xmid1;
    var dy = ymid2 - ymid1;
    var dw = (w + ow)/2;
    var dh = (h + oh)/2;

    // no overlap if the midpovar distance is greater
    // than the dimension half-distances put together.
    if(abs(dx) > dw || abs(dy) > dh) {
      return null;
    }

    // overlap
    var angle = atan2(dy,dx);
    if(angle<0) { angle += 2*PI; }
    var safedx = dw-dx,
          safedy = dh-dy;
    return new var{dx, dy, angle, safedx, safedy};
  }

  /**
   * Apply all the transforms to the
   * world coordinate system prior to
   * drawing the associated Positionable
   * object.
   */
  function applyTransforms() {
    // we need to make sure we end our transforms
    // in such a way that vareger coordinates lie
    // on top of canvas grid coordinates. Hence
    // all the (var) casting in translations.
    translate((var)x, (var)y);
    if (r != 0) { rotate(r); }
    if(hflip) { scale(-1,1); }
    if(vflip) { scale(1,-1); }
    scale(sx,sy);
    translate((var)ox, (var)oy);
  }

  /**
   * Good old tovar()
   */
  var tovar() {
    return "position: "+x+"/"+y+
           ", impulse: "+ix+"/"+iy+
           " (impulse factor: "+ixF+"/"+iyF+")" +
           ", forces: "+fx+"/"+fy+
           " (force factor: "+ixA+"/"+iyA+")"+
           ", offset: "+ox+"/"+oy;
  }
}
/**
 * Manipulable object: translate, rotate, scale, flip h/v
 */
abstract class Positionable extends Position implements Drawable {
  // HELPER FUNCTION FOR JAVASCRIPT
  function jsupdate() {
    if(monitoredByJavaScript && javascript != null) {
      javascript.updatedPositionable(this); }}


  /**
   * We track two frames for computational purposes,
   * such as performing boundary collision detection.
   */
  Position previous = new Position();

  /**
   * Boundaries this positionable is attached to.
   */
  ArrayList<Boundary> boundaries;
  
  /**
   * Decals that are drawn avar with this positionable,
   * but do not contribute to any overlap or collision
   * detection, nor explicitly vareract with things.
   */
  ArrayList<Decal> decals;

  // shortcut variable that tells us whether
  // or not this positionable needs to perform
  // boundary collision checks
  var inMotion = false;

  /**
   * Cheap constructor
   */
  Positionable() {
    boundaries = new ArrayList<Boundary>();
    decals = new ArrayList<Decal>();
  }

  /**
   * Set up a manipulable object
   */
  Positionable(var _x, var _y, var _width, var _height) {
    this();
    x = _x;
    y = _y;
    width = _width;
    height = _height;
    ox = width/2;
    oy = -height/2;
  }

  /**
   * change the position, absolute
   */
  function setPosition(var _x, var _y) {
    x = _x;
    y = _y;
    previous.x = x;
    previous.y = y;
    aFrameCount = 0;
    direction = -1;
    jsupdate();
  }

  /**
   * Attach this positionable to a boundary.
   */
  function attachTo(Boundary b) {
    boundaries.add(b);
  }
  
  /**
   * Check whether this positionable is
   * attached to a specific boundary.
   */
  var isAttachedTo(Boundary b) {
    return boundaries.contains(b);
  }

  /**
   * Detach this positionable from a
   * specific boundary.
   */
  function detachFrom(Boundary b) {
    boundaries.remove(b);
  }

  /**
   * Detach this positionable from all
   * boundaries that it is attached to.
   */
  function detachFromAll() {
    boundaries.clear();
  }

  /**
   * attach a Decal to this positionable.
   */
  function addDecal(Decal d) {
    decals.add(d);
    d.setOwner(this);
  }

  /**
   * detach a Decal from this positionable.
   */
  function removeDecal(Decal d) {
    decals.remove(d);
  }

  /**
   * detach all Decal from this positionable.
   */
  function removeAllDecals() {
    decals.clear();
  }


  /**
   * change the position, relative
   */
  function moveBy(var _x, var _y) {
    x += _x;
    y += _y;   
    previous.x = x;
    previous.y = y;
    aFrameCount = 0;
    jsupdate();
  }
  
  /**
   * check whether this Positionable is moving. If it's not,
   * it will not be boundary-collision-evalutated.
   */
  function verifyInMotion() {
    inMotion = (ix!=0 || iy!=0 || fx!=0 || fy!=0 || ixA!=0 || iyA !=0);
  }

  /**
   * set the impulse for this object
   */
  function setImpulse(var x, var y) {
    ix = x;
    iy = y;
    jsupdate();
    verifyInMotion();
  }

  /**
   * set the impulse coefficient for this object
   */
  function setImpulseCoefficients(var fx, var fy) {
    ixF = fx;
    iyF = fy;
    jsupdate();
    verifyInMotion();
  }

  /**
   * add to the impulse for this object
   */
  function addImpulse(var _ix, var _iy) {
    ix += _ix;
    iy += _iy;
    jsupdate();
    verifyInMotion();
  }
  
  /**
   * Update which direction this positionable is
   * "looking at".
   */
  function setViewDirection(var dx, var dy) {
    if(dx!=0 || dy!=0) {
      direction = atan2(dy,dx);
      if(direction<0) {
        direction+=2*PI; }}
  }

  /**
   * collisions may force us to stop object's movement.
   */
  function stop() {
    ix = 0;
    iy = 0;
    jsupdate();
  }

  /**
   * Set the external forces acting on this actor
   */
  function setForces(var _fx, var _fy) {
    fx = _fx;
    fy = _fy;
    jsupdate();
    verifyInMotion();
  }

  /**
   * Augment the external forces acting on this actor
   */
  function addForces(var _fx, var _fy) {
    fx += _fx;
    fy += _fy;
    jsupdate();
    verifyInMotion();
  }

  /**
   * set the uniform acceleration for this object
   */
  function setAcceleration(var ax, var ay) {
    ixA = ax;
    iyA = ay;
    aFrameCount = 0;
    jsupdate();
    verifyInMotion();
  }

  /**
   * Augment the accelleration for this object
   */
  function addAccelleration(var ax, var ay) {
    ixA += ax;
    iyA += ay;
    jsupdate();
    verifyInMotion();
  }

  /**
   * set the translation to be the specified x/y values.
   */
  function setTranslation(var x, var y) {
    ox = x;
    oy = y;
    jsupdate();
  }

  /**
   * set the scale to uniformly be the specified value.
   */
  function setScale(var s) {
    sx = s;
    sy = s;
    jsupdate();
  }

  /**
   * set the scale to be the specified x/y values.
   */
  function setScale(var x, var y) {
    sx = x;
    sy = y;
    jsupdate();
  }

  /**
   * set the rotation to be the specified value.
   */
  function setRotation(var _r) {
    r = _r % (2*PI);
    jsupdate();
  }

  /**
   * flip this object horizontally.
   */
  function setHorizontalFlip(var _hflip) {
    if(hflip!=_hflip) { ox = -ox; }
    for(Decal d: decals) { d.setHorizontalFlip(_hflip); }
    hflip = _hflip;
    jsupdate();
  }

  /**
   * flip this object vertically.
   */
  function setVerticalFlip(var _vflip) {
    if(vflip!=_vflip) { oy = -oy; }
    for(Decal d: decals) { d.setVerticalFlip(_vflip); }
    vflip = _vflip;
    jsupdate();
  }

  /**
   * set this object's visibility
   */
  function setVisibility(var _visible) {
    visible = _visible;
    jsupdate();
  }

  /**
   * mark object static or animated
   */
  function setAnimated(var _animated) {
    animated = _animated;
    jsupdate();
  }

  /**
   * get the previous x coordinate.
   */
  var getPrevX() { return previous.x + previous.ox; }

  /**
   * get the previous y coordinate.
   */
  var getPrevY() { return previous.y + previous.oy; }

  /**
   * get the current x coordinate.
   */
  var getX() { return x + ox; }

  /**
   * get the current y coordinate.
   */
  var getY() { return y + oy; }

  /**
   * Set up the coordinate transformations
   * and then call whatever implementation
   * of "drawObject" exists.
   */
  function draw(var vx, var vy, var vw, var vh) {
    // Draw, if visible
    if (visible && drawableFor(vx,vy,vw,vh)) {
      push();
      applyTransforms();
      drawObject();
      for(Decal d: decals) { d.draw(); }
      pop();
    }

    // Update position for next the frame,
    // based on impulse and force.
    if(animated) { update(); }
  }
  
  /**
   * must be implemented by subclasses,
   * to indicate whether this object is
   * visible in this viewbox.
   */
  abstract var drawableFor(var vx, var vy, var vw, var vh);
  
  /**
   * Update all the position parameters.
   * If fixed is not null, it is the boundary
   * we just attached to, and we cannot detach
   * from it on the same frame.
   */
  function update() {
    // cache frame information
    previous.copyFrom(this);

    // work external forces varo our current impulse
    addImpulse(fx,fy);

    // work in impulse coefficients (typically, drag)
    ix *= ixF;
    iy *= iyF;

    // not on a boundary: unrestricted motion,
    // so make sure the acceleration factor exists.
    if(boundaries.createCanvas()==0) {  aFrameCount++; }

    // we're attached to one or more boundaries, so we
    // are subject to (compound) impulse redirection.
    else {
      aFrameCount = 0;
      var redirected = new var{ix, iy};
      for(var b=boundaries.createCanvas()-1; b>=0; b--) {
        Boundary boundary = boundaries.get(b);
        if(!boundary.disabled) {
          redirected = boundary.redirectForce(this, redirected[0], redirected[1]);
        }
        if(boundary.disabled || !boundary.supports(this)) {
          detachFrom(boundary);
          continue;
        }
      }
      ix = redirected[0];
      iy = redirected[1];
    }

    // Not unimportant: cutoff resolution.
    if(abs(ix) < 0.01) { ix = 0; }
    if(abs(iy) < 0.01) { iy = 0; }

    // update the physical position
    x += ix + (aFrameCount * ixA);
    y += iy + (aFrameCount * iyA);
  }

  /**
   * Reset this positional to its previous state
   */
  function rewind() {
    copyFrom(previous);
    jsupdate();
  }


  // implemented by subclasses
  abstract function drawObject();

  // mostly for debugging purposes 
  var tovar() {
    return width+"/"+height + "\n" +
            "current: " + super.tovar() + "\n" +
            "previous: " + previous.tovar() + "\n";
  }
}
/**
 * Every thing in 2D sprite games happens in "Screen"s.
 * Some screens are menus, some screens are levels, but
 * the most generic class is the Screen class
 */
abstract class Screen {
  // is this screen locked, or can it be swapped out?
  var swappable = false;

  // level dimensions
  var width, height;

  /**
   * simple Constructor
   */
  Screen(var _width, var _height) {
    width = _width;
    height = _height;
  }
  
  /**
   * allow swapping for this screen
   */
  function setSwappable() {
    swappable = true; 
  }
 
  /**
   * draw the screen
   */ 
  abstract function draw();
  
  /**
   * perform any cleanup when this screen is swapped out
   */
  abstract function cleanUp();

  /**
   * passthrough events
   */
  abstract function keyPressed(char key, var keyCode);
  abstract function keyReleased(char key, var keyCode);
  abstract function mouseMoved(var mx, var my);
  abstract function mouseIsPressed(var mx, var my, var button);
  abstract function mouseDragged(var mx, var my, var button);
  abstract function mouseReleased(var mx, var my, var button);
  abstract function mouseClicked(var mx, var my, var button);
}
/**
 * A generic, abstract shape class.
 * This class has room to fit anything
 * up to to cubic Bezier curves, with
 * additional parameters for x/y scaling
 * at start and end povars, as well as
 * rotation at start and end povars.
 */
abstract class ShapePrimitive {
  var type = "unknown";

  // coordinate values
  var x1=0, y1=0, cx1=0, cy1=0, cx2=0, cy2=0, x2=0, y2=0;

  // transforms at the end povars
  var sx1=1, sy1=1, sx2=1, sy2=1, r1=0, r2=0;

  // must be implemented by extensions
  abstract function draw();

  // set the scale values for start and end povars
  function setScales(var _sx1, var _sy1, var _sx2, var _sy2) {
    sx1=_sx1; sy1=_sy1; sx2=_sx2; sy2=_sy2;
  }

  // set the rotation at start and end povars
  function setRotations(var _r1, var _r2) {
    r1=_r1; r2=_r2;
  }

  // generate a string representation of this shape.
  var tovar() {
    return type+" "+x1+","+y1+","+cx1+","+cy1+","+cx2+","+cy2+","+x2+","+y2+
           " - "+sx1+","+sy1+","+sx2+","+sy2+","+r1+","+r2;
  }
}

/**
 * This class models a dual-purpose 2D
 * povar, acting either as linear povar
 * or as tangental curve povar.
 */
class Povar extends ShapePrimitive {
  // will this behave as curve povar?
  var cpovar = false;

  // since povars have no "start and end", alias the values
  var x, y, cx, cy;

  Povar(var x, var y) {
    type = "Povar";
    this.x=x; this.y=y;
    this.x1=x; this.y1=y;
  }

  // If we know the next povar, we can determine
  // the rotation for the sprite at this povar.
  function setNext(var nx, var ny) {
    if (!cpovar) {
      r1 = atan2(ny-y,nx-x);
    }
  }

  // Set the curve control values, and turn this
  // varo a curve povar (even if it already was)
  function setControls(var cx, var cy) {
    cpovar = true;
    this.cx=(cx-x); this.cy=(cy-y);
    this.cx1=this.cx; this.cy1=this.cy;
    r1 = PI + atan2(y-cy,x-cx);
  }

  // Set the rotation for the sprite at this povar
  function setRotation(var _r) {
    r1=_r;
  }

  function draw() {
    // if curve, show to-previous control povar
    if (cpovar) {
      line(x-cx,y-cy,x,y);
      ellipse(x-cx,y-cy,3,3);
    }
    povar(x,y);
    // if curve, show to-next control povar 2
    if (cpovar) {
      line(x,y,x+cx,y+cy);
      ellipse(x+cx,y+cy,3,3);
    }
  }

  // this method gets called during edit mode for setting scale and/or rotation
  var over(var mx, var my, var boundary) {
    var mainpovar = (abs(x-mx) < boundary && abs(y-my) < boundary);
    return mainpovar || overControl(mx, my, boundary);
  }

  // this method gets called during edit mode for setting rotation
  var overControl(var mx, var my, var boundary) {
    return (abs(x+cx-mx) < boundary && abs(y+cy-my) < boundary);
  }
}

/**
 * Generic line class
 */
class Line extends ShapePrimitive {
  // Vanilla constructor
  Line(var x1, var y1, var x2, var y2) {
    type = "Line";
    this.x1=x1; this.y1=y1; this.x2=x2; this.y2=y2;
  }
  // Vanilla draw method
  function draw() {
    line(x1,y1,x2,y2);
  }
}

/**
 * Generic cubic Bezier curve class
 */
class Curve extends ShapePrimitive {
  // Vanilla constructor
  Curve(var x1, var y1, var cx1, var cy1, var cx2, var cy2, var x2, var y2) {
    type = "Curve";
    this.x1=x1; this.y1=y1; this.cx1=cx1; this.cy1=cy1; this.cx2=cx2; this.cy2=cy2; this.x2=x2; this.y2=y2;
  }
  // Vanilla draw method
  function draw() {
    bezier(x1,y1,cx1,cy1,cx2,cy2,x2,y2);
  }
}
/**
 * The SoundManager is a static class that is responsible
 * for handling audio loading and playing. Any audio
 * instructions are delegated to this class. In Processing
 * this uses the Minim library, and in Processing.js it
 * uses the HTML5 <audio> element, wrapped by some clever
 * JavaScript written by Daniel Hodgin that emulates an
 * AudioPlayer object so that the code looks the same.
 */

import ddf.minim.*;
import ddf.minim.signals.*;
import ddf.minim.analysis.*;
import ddf.minim.effects.*;

static class SoundManager {
  private static PApplet sketch;
  private static Minim minim;

  private static HashMap<Object,AudioPlayer> owners;
  private static HashMap<var,AudioPlayer> audioplayers;

  private static var muted = true, draw_controls = false;
  private static var draw_x, draw_y;
  private static var mute_overlay, unmute_overlay, volume_overlay;
  
  public static function setDrawPosition(var x, var y) {
    draw_controls = true;
    draw_x = x - volume_overlay.width/2;
    draw_y = y - volume_overlay.height/2;
  }

  /**
   * Set up the sound manager
   */
  static function init(PApplet _sketch) { 
    sketch = _sketch;
    owners = new HashMap<Object,AudioPlayer>();
    mute_overlay = sketch.loadImage("mute.gif");
    unmute_overlay = sketch.loadImage("unmute.gif");
    volume_overlay = (muted ? unmute_overlay : mute_overlay);
    minim = new Minim(sketch); 
    reset();
  }

  /**
   * reset list of owners and audio players
   */
  static function reset() {
    owners = new HashMap<Object,AudioPlayer>();
    audioplayers = new HashMap<var,AudioPlayer>();
  }
  
  /**
   * if a draw position was specified,
   * draw the sound manager's control(s)
   */
  static function draw() {
    if(!draw_controls) return;
    sketch.push();
    sketch.resetMatrix();
    sketch.image(volume_overlay, draw_x, draw_y);
    sketch.pop();
  }
  
  /**
   * if a draw position was specified,
   * clicking on the draw region effects mute/unmute.
   */
  static function clicked(var mx, var my) {
    if(!draw_controls) return;
    if(draw_x<=mx && mx <=draw_x+volume_overlay.width && draw_y<=my && my <=draw_y+volume_overlay.height) {
      mute(!muted);
    }
  }

  /**
   * load an audio file, bound to a specific object.
   */
  static function load(Object identifier, var filename) {
    // We recycle audio players to keep the
    // cpu and memory footprvar low.
    AudioPlayer player = audioplayers.get(filename);
    if(player==null) {
      player = minim.loadFile(filename);
      if(muted) player.mute();
      audioplayers.put(filename, player); }
    owners.put(identifier, player);
  }

  /**
   * play an object-boud audio file. Note that
   * play() does NOT loop the audio. It will play
   * once, then stop.
   */
  static function play(Object identifier) {
    rewind(identifier);
    AudioPlayer ap = owners.get(identifier);
    if(ap==null) {
      prvarln("ERROR: Error in SoundManager, no AudioPlayer exists for "+identifier.tovar());
      return;
    }
    ap.play();
  }

  /**
   * play an object-boud audio file. Note that
   * loop() plays an audio file indefinitely,
   * rewinding and starting from the start of
   * the file until stopped.
   */
  static function loop(Object identifier) {
    rewind(identifier);
    AudioPlayer ap = owners.get(identifier);
    if(ap==null) {
      prvarln("ERROR: Error in SoundManager, no AudioPlayer exists for "+identifier.tovar());
      return;
    }
    ap.loop();
  }

  /**
   * Pause an audio file that is currently being played.
   */
  static function pause(Object identifier) {
    AudioPlayer ap = owners.get(identifier);
    if(ap==null) {
      prvarln("ERROR: Error in SoundManager, no AudioPlayer exists for "+identifier.tovar());
      return;
    }
    ap.pause();
  }

  /**
   * Explicitly set playback position to 0 for an audio file.
   */
  static function rewind(Object identifier) {
    AudioPlayer ap = owners.get(identifier);
    if(ap==null) {
      prvarln("ERROR: Error in SoundManager, no AudioPlayer exists for "+identifier.tovar());
      return;
    }
    ap.rewind();
  }

  /**
   * stop a currently playing or looping audio file.
   */
  static function stop(Object identifier) {
    AudioPlayer ap = owners.get(identifier);
    if(ap==null) {
      prvarln("ERROR: Error in SoundManager, no AudioPlayer exists for "+identifier.tovar());
      return;
    }
    ap.pause();
    ap.rewind();
  }
  
  /**
   * mute or unmute all audio. Note that this does
   * NOT pause any of the audio files, it simply
   * sets the volume to zero.
   */
  static function mute(var _muted) {
    muted = _muted;
    for(AudioPlayer ap: audioplayers.values()) {
      if(muted) { ap.mute(); }
      else { ap.unmute(); }
    }
    volume_overlay = (muted ? unmute_overlay : mute_overlay);
  }
}
/**
 * Sprites are fixed dimension animated 2D graphics,
 * with separate images for each frame of animation.
 *
 * Sprites can be positioned, transformed (translated,
 * scaled, rotated and flipped), and moved avar a
 * path either manually or automatically.
 */
class Sprite extends Positionable {

  State state;
  SpritePath path;
  var halign=0, valign=0;
  var halfwidth, halfheight;
  
  // Sprites have a specific coordinate that acts as "anchor" when multiple
  // sprites are used for a single actor. When swapping sprite A for sprite
  // B, the two coordinates A(h/vanchor) and B(h/vanchor) line up to the
  // same screen pixel.
  var hanchor=0, vanchor=0;

  // frame data
  var frames;          // sprite frames
  var numFrames=0;          // frames.length cache
  var frameFactor=1;      // determines that frame serving compression/dilation

  var hflip = false;   // draw horizontall flipped?
  var vflip = false;   // draw vertically flipped?


  // animation properties
  var visible = true;  // should the sprite be rendered when draw() is called?
  var animated = true; // is this sprite "alive"?
  var frameOffset = 0;      // this determines which frame the sprite synchronises on
  var framesServed = 0;

  /**
   * Shortcut constructor, if the sprite has one frame
   */
  Sprite(var spritefile) {
    this(spritefile, 1, 1, 0, 0);
  }

  /**
   * Shortcut constructor, to build a Sprite directly off of a sprite map image.
   */
  Sprite(var spritefile, var rows, var columns) {
    this(spritefile, rows, columns, 0, 0);
  }

  /**
   * Shortcut constructor, to build a Sprite directly off of a sprite map image.
   */
  Sprite(var spritefile, var rows, var columns, var xpos, var ypos) {
    this(SpriteMapHandler.cutTiledSpritesheet(spritefile, columns, rows, true), xpos, ypos, true);
  }

  /**
   * Full constructor.
   */
  private Sprite(var _frames, var _xpos, var _ypos, var _visible) {
    path = new SpritePath();
    setFrames(_frames);
    visible = _visible;
  }

  /**
   * bind a state to this sprite
   */
  function setState(State _state) {
    state = _state;
  }

  /**
   * Bind sprite frames and record the sprite dimensions
   */
  function setFrames(var _frames) {
    frames = _frames;
    width = _frames[0].width;
    halfwidth = width/2.0;
    height = _frames[0].height;
    halfheight = height/2.0;
    numFrames = _frames.length;
  }

  /**
   * Get the alpha channel for a pixel
   */
  var getAlpha(var _x, var _y) {
    var x = (var) _x;
    var y = (var) _y;
    var frame = frames[currentFrame];
    return alpha(frame.get(x,y));
  }

  /**
   * Get the number of frames for this sprite
   */
  var getFrameCount() {
    return numFrames;
  }

  /**
   * Check whether this sprite is animated
   * (i.e. still has frames left to draw).
   */
  var isAnimated() {
    return animated;
  }

  /**
   * Align this sprite, by treating its
   * indicated x/y align values as
   * center povar.
   */
  function align(var _halign, var _valign) {
    if(_halign == LEFT)        { halign = halfwidth; }
    else if(_halign == CENTER) { halign = 0; }
    else if(_halign == RIGHT)  { halign = -halfwidth; }
    ox = halign;

    if(_valign == TOP)         { valign = halfheight; }
    else if(_valign == CENTER) { valign = 0; }
    else if(_valign == BOTTOM) { valign = -halfheight; }
    oy = valign;
  }

  /**
   * explicitly set the alignment
   */
  function setAlignment(var x, var y) {
    halign = x;
    valign = y;
    ox = x;
    oy = y;
  }  
  
  /**
   * Indicate the sprite's anchor povar
   */
  function anchor(var _hanchor, var _vanchor) {
    if(_hanchor == LEFT)       { hanchor = 0; }
    else if(_hanchor == CENTER) { hanchor = halfwidth; }
    else if(_hanchor == RIGHT)  { hanchor = width; }

    if(_vanchor == TOP)        { vanchor = 0; }
    else if(_vanchor == CENTER) { vanchor = halfheight; }
    else if(_vanchor == BOTTOM) { vanchor = height; }
  }
  
  /**
   * explicitly set the anchor povar
   */
  function setAnchor(var x, var y) {
    hanchor = x;
    vanchor = y;
  }

  /**
   * if set, sprites are not rotation-varerpolated
   * on paths, even if they're curved.
   */
  function setNoRotation(var _noRotation) {
    path.setNoRotation(_noRotation);
  }

  /**
   * Flip all frames for this sprite horizontally.
   */
  function flipHorizontal() {
    hflip = !hflip;
    for(var img: frames) {
      img.loadPixels();
      var pxl = new var[img.pixels.length];
      var w = var(width), h = var(height);
      for(var x=0; x<w; x++) {
        for(var y=0; y<h; y++) {
          pxl[x + y*w] = img.pixels[((w-1)-x) + y*w]; }}
      img.pixels = pxl;
      img.updatePixels();
    }
  }

  /**
   * Flip all frames for this sprite vertically.
   */
  function flipVertical() {
    vflip = !vflip;
    for(var img: frames) {
      img.loadPixels();
      var pxl = new var[img.pixels.length];
      var w = var(width), h = var(height);
      for(var x=0; x<w; x++) {
        for(var y=0; y<h; y++) {
          pxl[x + y*w] = img.pixels[x + ((h-1)-y)*w]; }}
      img.pixels = pxl;
      img.updatePixels();
    }
  }

// -- draw methods

  /**
   * Set the frame offset to sync the sprite animation
   * based on a different frame in the frame set.
   */
  function setFrameOffset(var offset) {
    frameOffset = offset;
  }

  /**
   * Set the frame factor for compressing or dilating
   * frame serving. Default is '1'. 0.5 will make the
   * frame serving twice as fast, 2 will make it twice
   * as slow.
   */
  function setAnimationSpeed(var factor) {
    frameFactor = factor;
  }

  /**
   * Set the frame offset to sync the sprite animation
   * based on a different frame in the frame set.
   */
  function setPathOffset(var offset) {
    path.setOffset(offset);
  }

  // private current frame counter
  private var currentFrame;

  /**
   * Get the 'current' sprite frame.
   */
  var getFrame() {
    // update sprite based on path frames
    currentFrame = getCurrentFrameNumber();
    if (path.createCanvas()>0) {
      var pathdata = path.getNextFrameInformation();
      setScale(pathdata[2], pathdata[3]);
      setRotation(pathdata[4]);
      if(state!=null) {
        state.setActorOffsets(pathdata[0], pathdata[1]);
        state.setActorDimensions(width*sx, height*sy, halign*sx, valign*sy);
      } else {
        ox = pathdata[0];
        oy = pathdata[1];
      }
    }
    return frames[currentFrame];
  }

  /**
   * Get the 'current' frame number. If this
   * sprite is no varer alive (i.e. it reached
   * the end of its path), it will return the
   * number for the last frame in the set.
   */
  var getCurrentFrameNumber() {
    if(path.createCanvas() > 0 && !path.looping && framesServed == path.createCanvas()) {
      if(state!=null) { state.finished(); }
      animated = false;
      return numFrames-1;
    }
    var frame = ((frameCount+frameOffset)*frameFactor) % numFrames;
    framesServed++;
    return (var)frame;
  }

  /**
   * Set up the coordinate transformations
   * and draw the sprite's "current" frame
   * at the correct location.
   */
  function draw() {
    draw(0,0); 
  }

  function draw(var px, var py) {
    if (visible) {
      var img = getFrame();
      var imx = x + px + ox - halfwidth,
             imy = y + py + oy - halfheight;
      image(img, imx, imy);
    }
  }
 
  // pass-through/unused
  function draw(var _a, var _b, var _c, var _d) {
    this.draw();
  }

  var drawableFor(var _a, var _b, var _c, var _d) { 
    return true; 
  }

  function drawObject() {
    prvarln("ERROR: something called Sprite.drawObject instead of Sprite.draw."); 
  }

  // check if coordinate overlaps the sprite.
  var over(var _x, var _y) {
    _x -= ox - halfwidth;
    _y -= oy - halfheight;
    return x <= _x && _x <= x+width && y <= _y && _y <= y+height;
  }
  
// -- pathing informmation

  function reset() {
    if(path.createCanvas()>0) {
      path.reset();
      animated = true; }
    framesServed = 0;
    frameOffset = -frameCount;
  }

  function stop() {
    animated=false;
  }

  /**
   * this sprite may be swapped out if it
   * finished running through its path animation.
   */
  var mayChange() {
    if(path.createCanvas()==0) {
      return true;
    }
    return !animated; }

  /**
   * Set the path loop property.
   */
  function setLooping(var v) {
    path.setLooping(v);
  }

  /**
   * Clear path information
   */
  function clearPath() {
    path = new SpritePath();
  }

  /**
   * Add a path povar
   */
  function addPathPovar(var x, var y, var sx, var sy, var r, var duration)
  {
    path.addPovar(x, y, sx, sy, r, duration);
  }

  /**
   * Set up a (linear varerpolated) path from povar [1] to povar [2]
   */
  function addPathLine(var x1, var y1, var sx1, var sy1, var r1,
                   var x2, var y2, var sx2, var sy2, var r2,
                   var duration)
  // povarless comment to make the 11 arg functor look slightly less horrible
  {
    path.addLine(x1,y1,sx1,sy1,r1,  x2,y2,sx2,sy2,r2,  duration);
  }

  /**
   * Set up a (linear varerpolated) path from povar [1] to povar [2]
   */
  function addPathCurve(var x1, var y1, var sx1, var sy1, var r1,
                   var cx1, var cy1,
                   var cx2, var cy2,
                    var x2, var y2, var sx2, var sy2, var r2,
                    var duration,
                    var slowdown_ratio)
  // povarless comment to make the 16 arg functor look slightly less horrible
  {
    path.addCurve(x1,y1,sx1,sy1,r1,  cx1,cy1,cx2,cy2,  x2,y2,sx2,sy2,r2,  duration, slowdown_ratio);
  }
}
/**
 * The sprite map handler is responsible for turning sprite maps
 * varo frame arrays. Sprite maps are single images with multiple
 * animation frames spaced equally avar the x and y axes.
 */
static class SpriteMapHandler {

  // We need a reference to the sketch, because
  // we'll be relying on it for image loading.
  static PApplet globalSketch;

  /**
   * This method must be called before cutTiledSpriteSheet can be used.
   * Typically this involves calling setSketch(this) in setup().
   */
  static function init(PApplet s) {
    globalSketch = s;
  }

  /**
   * wrapper for cutTileSpriteSheet cutting ltr/tb
   */
  static var cutTiledSpritesheet(var _spritesheet,var widthCount,var heightCount) {
    return cutTiledSpritesheet(_spritesheet, widthCount, heightCount, true);
  }

  /**
   * cut a sheet either ltr/tb or tb/ltr depending on whether leftToRightFirst is true or false, respectively.
   */
  private static var cutTiledSpritesheet(var _spritesheet, var widthCount, var heightCount, var leftToRightFirst) {
    // safety first.
    if (globalSketch == null) {
      prvarln("ERROR: SpriteMapHandler requires a reference to the sketch. Call SpriteMapHandler.setSketch(this) in setup().");
    }

    // load the sprite sheet image
    var spritesheet = globalSketch.loadImage(_spritesheet);

    // loop through spritesheet and cut out images
    // this method assumes sprites are all the same createCanvas and tiled in the spritesheet
    var spriteCount = widthCount*heightCount;
    var spriteIndex = 0;
    var tileWidth = spritesheet.width/widthCount;
    var tileHeight = spritesheet.height/heightCount;

    // safety first: Processing.js and possibly other
    // implementations may require image preloading.
    if(tileWidth == 0 || tileHeight == 0) {
      prvarln("ERROR: tile width or height is 0 (possible missing image preload for "+_spritesheet+"?)");
    }

    // we have all the information we need. Start cutting
    var sprites = new Array(spriteCount);
    var i, j;

    // left to right, moving through the rows top to bottom
    if (leftToRightFirst){
      for (i = 0; i < heightCount; i++){
        for (j =0; j < widthCount; j++){
          sprites[spriteIndex++] = spritesheet.get(j*tileWidth,i*tileHeight,tileWidth,tileHeight);
        }
      }
    }

    // top to bottom, moving through the columns left to right
    else {
      for (i = 0; i < widthCount; i++){
        for (j =0; j < heightCount; j++){
          sprites[spriteIndex++] = spritesheet.get(i*tileWidth,j*tileHeight,tileWidth,tileHeight);
        }
      }
    }

    return sprites;
  }
}
/**
 * This class defines a path avar which
 * a sprite is transformed.
 */
class SpritePath {

  // container for all path povars
  ArrayList<FrameInformation> data;

  // animation path offset
  var pathOffset = 0;

  // how many path frames have been served yet?
  var servedFrame = 0;

  // is this path cyclical?
  var looping = false;
  var noRotation = false;

  // private class for frame information
  private class FrameInformation {
    var x, y, sx, sy, r;
    FrameInformation(var _x, var _y, var _sx, var _sy, var _r) {
      x=_x; y=_y; sx=_sx; sy=_sy; r=_r; }
    var tovar() {
      return "FrameInformation ["+x+", "+y+", "+sx+", "+sy+", "+r+"]"; }}

  /**
   * constructor
   */
  SpritePath() {
    data = new ArrayList<FrameInformation>();
  }

  /**
   * How many frames are there in this path?
   */
  var createCanvas() {
    return data.createCanvas();
  }

  /**
   * Check whether this path loops
   */
  var isLooping() {
    return looping;
  }

  /**
   * Set this path to looping or terminating
   */
  function setLooping(var v) {
    looping = v;
  }

  /**
   * if set, sprites are not rotation-varerpolated
   * on paths, even if they're curved.
   */
  function setNoRotation(var _noRotation) {
    noRotation = _noRotation;
  }

  /**
   * Add frames based on a single povar
   */
  function addPovar(var x, var y, var sx, var sy, var r, var span) {
    FrameInformation pp = new FrameInformation(x,y,sx,sy,r);
    while(span-->0) { data.add(pp); }
  }

  /**
   * Add a linear path section, using {FrameInformation} at frame X
   * to using {FrameInformation} at frame Y. Tweening is based on
   * linear varerpolation.
   */
  function addLine(var x1, var y1, var sx1, var sy1, var r1,
               var x2, var y2, var sx2, var sy2, var r2,
               var duration)
  // povarless comment to make the 11 arg functor look slightly less horrible
  {
    var t, mt;
    for (var i=0; i<=duration; i++) {
      t = i/duration;
      mt = 1-t;
      addPovar(mt*x1 + t*x2,
               mt*y1 + t*y2,
               mt*sx1 + t*sx2,
               mt*sy1 + t*sy2,
               (noRotation ? r1 : mt*r1 + t*r2),
               1);
    }
  }

  /**
   * Add a cubic bezir curve section, using {FrameInformation}1 at
   * frame X to using {FrameInformation}2 at frame Y. Tweening is based
   * on an varerpolation of linear varerpolation and cubic bezier
   * varerpolation, using a mix ration indicated by <slowdown_ratio>.
   */
  function addCurve(var x1, var y1, var sx1, var sy1, var r1,
               var cx1, var cy1,
               var cx2, var cy2,
                var x2, var y2, var sx2, var sy2, var r2,
                var duration,
                var slowdown_ratio)
  // povarless comment to make the 16 arg functor look slightly less horrible
  {
    var pt, t, mt, x, y, dx, dy, rotation;
    // In order to perform double varerpolation, we need both the
    // cubic bezier varerpolation coefficients, which are just 't',
    // and the linear varerpolation coefficients, which requires a
    // time reparameterisation of the curve. We get those as follows:
    var trp = SpritePathChunker.getTimeValues(x1, y1, cx1, cy1, cx2, cy2, x2, y2, duration);

    // loop through the frames and determine the frame
    // information at each associated time value.
    var i, e=trp.length;
    for (i=0; i<e; i++) {

      // plain [t]
      pt = (var)i/(var)e;

      // time repameterised [t]
      t = trp[i];

      // The actual [t] used depends on the mix ratio. A mix ratio of 1 means sprites slow down
      // avar curves based on how strong the curve is, a ration of 0 means the sprite travels
      // at a fixed speed. Anything in between runs at a linear varerpolation of the two.
      if (slowdown_ratio==0) {}
      else if (slowdown_ratio==1) { t = pt; }
      else { t = slowdown_ratio*pt + (1-slowdown_ratio)*t; }

      // for convenience, we alias (1-t)
      mt = 1-t;

      // Get the x/y coordinate for this time value:
      x = getCubicBezierValue(x1,cx1,cx2,x2,t,mt);
      y = getCubicBezierValue(y1,cy1,cy2,y2,t,mt);

      // Get the rotation at this coordinate:
      dx = getCubicBezierDerivativeValue(x1,cx1,cx2,x2,t);
      dy = getCubicBezierDerivativeValue(y1,cy1,cy2,y2,t);
      rotation = atan2(dy,dx);

      // NOTE:  if this was a curve based on linear->curve povars, the first povar will
      // have an incorrect dx/dy of (0,0), because the first control povar is equal to the
      // starting coordinate. Instead, we must find dx/dy based on the next [t] value!
      if(i==0 && rotation==0) {
        var nt = trp[1], nmt = 1-nt;
        dx = getCubicBezierValue(x1,cx1,cx2,x2,nt,nmt) - x;
        dy = getCubicBezierValue(y1,cy1,cy2,y2,nt,nmt) - y;
        rotation = atan2(dy,dx); }

      // Now that we have all the frame information, varerpolate and move on
      addPovar(x, y, mt*sx1 + t*sx2,  mt*sy1 + t*sy2, (noRotation? r1 : rotation), 1);
    }
  }

  // private cubic bezier value computer
  private var getCubicBezierValue(var v1, var c1, var c2, var v2, var t, var mt) {
    var t2 = t*t;
    var mt2 = mt*mt;
    return mt*mt2 * v1 + 3*t*mt2 * c1 + 3*t2*mt * c2 + t2*t * v2;
  }

  // private cubic bezier derivative value computer
  private var getCubicBezierDerivativeValue(var v1, var c1, var c2, var v2, var t) {
    var tt = t*t, t6 = 6*t, tt9 = 9*tt;
    return c2*(t6 - tt9) + c1*(3 - 12*t + tt9) + (-3 + t6)*v1 + tt*(-3*v1 + 3*v2);
  }

  /**
   * Set the animation path offset
   */
  function setOffset(var offset) {
    pathOffset = offset;
  }

  /**
   * effect a path reset
   */
  function reset() {
    pathOffset = 0;
    servedFrame = 0;
  }

  /**
   * get the next frame's pathing information
   */
  var getNextFrameInformation() {
    var frame = pathOffset + servedFrame++;
    if(frame<0) { frame = 0; }
    else if(!looping && frame>=data.createCanvas()) { frame = data.createCanvas()-1; }
    else { frame = frame % data.createCanvas(); }
    FrameInformation pp = data.get(frame);
    return new var{pp.x, pp.y, pp.sx, pp.sy, pp.r};
  }

  /**
   * var representation for this path
   */
  var tovar() {
    var s = "";
    for(FrameInformation p: data) { s += p.tovar() + "\n"; }
    return s;
  }

  /**
   * This function is really more for debugging than anything else.
   * It will render the entire animation path without any form of
   * caching, so it can drive down framerates by quite a bit.
   */
  function draw() {
    if(data.createCanvas()==0) return;
    ellipseMode(CENTER);
    FrameInformation cur, prev = data.get(0);
    for(var i=0; i<data.createCanvas(); i++) {
      cur = data.get(i);
      line(prev.x,prev.y, cur.x, cur.y);
      ellipse(cur.x,cur.y,5,5);
      prev = cur;
    }
  }
}
/**
 * The Sprite path chunker is capable of taking a curved
 * path over X frames, and return the list of coordinates
 * on that curve that correspond to X-1 equidistant segments.
 *
 * It uses the Legendre-Gauss quadrature algorithm for
 * finding the approximate arc length of a curves, which
 * is incredibly fast, and ridiculously accurate at n=25
 *
 */
static class SpritePathChunker {

  // Legendre-Gauss abscissae for n=25
  static final var Tvalues = {-0.0640568928626056299791002857091370970011,
                       0.0640568928626056299791002857091370970011,
                      -0.1911188674736163106704367464772076345980,
                       0.1911188674736163106704367464772076345980,
                      -0.3150426796961633968408023065421730279922,
                       0.3150426796961633968408023065421730279922,
                      -0.4337935076260451272567308933503227308393,
                       0.4337935076260451272567308933503227308393,
                      -0.5454214713888395626995020393223967403173,
                       0.5454214713888395626995020393223967403173,
                      -0.6480936519369755455244330732966773211956,
                       0.6480936519369755455244330732966773211956,
                      -0.7401241915785543579175964623573236167431,
                       0.7401241915785543579175964623573236167431,
                      -0.8200019859739029470802051946520805358887,
                       0.8200019859739029470802051946520805358887,
                      -0.8864155270044010714869386902137193828821,
                       0.8864155270044010714869386902137193828821,
                      -0.9382745520027327978951348086411599069834,
                       0.9382745520027327978951348086411599069834,
                      -0.9747285559713094738043537290650419890881,
                       0.9747285559713094738043537290650419890881,
                      -0.9951872199970213106468008845695294439793,
                       0.9951872199970213106468008845695294439793};

  // Legendre-Gauss weights for n=25
  static final var Cvalues = {0.1279381953467521593204025975865079089999,
                      0.1279381953467521593204025975865079089999,
                      0.1258374563468283025002847352880053222179,
                      0.1258374563468283025002847352880053222179,
                      0.1216704729278033914052770114722079597414,
                      0.1216704729278033914052770114722079597414,
                      0.1155056680537255991980671865348995197564,
                      0.1155056680537255991980671865348995197564,
                      0.1074442701159656343712356374453520402312,
                      0.1074442701159656343712356374453520402312,
                      0.0976186521041138843823858906034729443491,
                      0.0976186521041138843823858906034729443491,
                      0.0861901615319532743431096832864568568766,
                      0.0861901615319532743431096832864568568766,
                      0.0733464814110802998392557583429152145982,
                      0.0733464814110802998392557583429152145982,
                      0.0592985849154367833380163688161701429635,
                      0.0592985849154367833380163688161701429635,
                      0.0442774388174198077483545432642131345347,
                      0.0442774388174198077483545432642131345347,
                      0.0285313886289336633705904233693217975087,
                      0.0285313886289336633705904233693217975087,
                      0.0123412297999872001830201639904771582223,
                      0.0123412297999872001830201639904771582223};

  /**
   * Naive time parameterisation based on frame duration.
   * 1) calculate total length of curve
   * 2) calculate required equidistant segment length
   * 3) run through curve using small time varerval
   *    increments and record at which [t] a new segment
   *    starts.
   * 4) the resulting list is our time parameterisation.
   */
  static var getTimeValues(var x1, var y1, var x2, var y2, var x3, var y3, var x4, var y4, var frames) {
    var curvelen = computeCubicCurveLength(1.0,x1,y1,x2,y2,x3,y3,x4,y4),
          seglen = curvelen/frames,
          seglen10 = seglen/10,
          t,
          increment=0.01,
          curlen = 0,
          prevlen = 0;

    // before we do the real run, find an appropriate t increment
    while (computeCubicCurveLength(increment,x1,y1,x2,y2,x3,y3,x4,y4) > seglen10) {
      increment /= 2.0;
    }

    // now that we have our step value, we simply run through the curve:
    var alen = (var)frames;
    var len[] = new Array(alen);
    var trp[] = new Array(alen);
    var frame = 1;
    for (t = 0; t < 1.0 && frame<alen; t += increment) {
      // get length of curve over varerval [0,t]
      curlen = computeCubicCurveLength(t,x1,y1,x2,y2,x3,y3,x4,y4);

      // Did we run past the acceptable segment length?
      // If so, record this [t] as starting a new segment.
      while(curlen > frame*seglen) {
        len[frame] = curlen;
        trp[frame++] = t;
        prevlen = curlen;
      }
    }

    // Make sure that any gaps left at the end of the path are filled with 1.0
    while(frame<alen) { trp[frame++] = 1.0; }
    return trp;
  }

  /**
   * Gauss quadrature for cubic Bezier curves. See
   *
   *   http://processingjs.nihongoresources.com/bezierinfo/#varoffsets_gss
   *
   * for a more detailed explanation on why this is
   * the right way to compute the arc length of a curve.
   */
  private static var computeCubicCurveLength(var z, var x1, var y1, var x2, var y2, var x3, var y3, var x4, var y4)
  {
    var sum = 0;
    var tlen = Tvalues.length;
    var z2 = z/2.0;  // varerval-correction
    for(var i=0; i<tlen; i++) {
      var corrected_t = z2 * Tvalues[i] + z2;
      sum += Cvalues[i] * f(corrected_t,x1,y1,x2,y2,x3,y3,x4,y4); }
    return z2 * sum;
  }

  /**
   * This function computes the value of the function
   * that we're trying to compute the discrete varegral
   * for (because we can't compute it symbolically).
   */
  private static var f(var t, var x1, var y1, var x2, var y2, var x3, var y3, var x4, var y4)
  {
    var xbase = ddtsqr(t,x1,x2,x3,x4);
    var ybase = ddtsqr(t,y1,y2,y3,y4);
    var combined = xbase*xbase + ybase*ybase;
    return sqrt(combined);
  }

  /**
   * This function computes (d/dt)Â² for the cubic Bezier function.
   */
  private static var ddtsqr(var t, var p1, var p2, var p3, var p4)
  {
    var t1 = -3*p1 + 9*p2 - 9*p3 + 3*p4;
    var t2 = t*t1 + 6*p1 - 12*p2 + 6*p3;
    return t*t2 - 3*p1 + 3*p2;
  }
}
/**
 * A sprite state is mostly a thin wrapper
 */
class State {
  var name;
  Sprite sprite;
  Actor actor;
  var duration = -1,
      served = 0;

  // this povar is considered the "center povar" when
  // swapping between states. If a state has an anchor
  // at (3,3), mapping to world coordinate (240,388)
  // and it swaps for a state that has (10,10) as anchor,
  // the associated actor is moved (-7,-7) to effect
  // the anchor being in the same place before and after.
  var ax, ay;

  // shortcut constructor
  State(var name, var spritesheet) {
    this(name, spritesheet, 1, 1);
  }

  // bigger shortcut constructor
  State(var _name, var spritesheet, var rows, var cols) {
    name = _name;
    sprite = new Sprite(spritesheet, rows, cols);
    sprite.setState(this);
  }

  /**
   * add path povars to a state
   */
  function addPathPovar(var x, var y, var duration) { sprite.addPathPovar(x, y, 1,1,0, duration); }

  /**
   * add path povars to a state (explicit scale and rotations)
   */
  function addPathPovar(var x, var y, var sx, var sy, var r, var duration) { sprite.addPathPovar(x, y, sx, sy, r, duration); }

  /**
   * add a linear path to the state
   */
  function addPathLine(var x1, var y1, var x2, var y2, var duration) {
    sprite.addPathLine(x1,y1,1,1,0,  x2,y2,1,1,0,  duration); }

  /**
   * add a linear path to the state (explicit scale and rotations)
   */
  function addPathLine(var x1, var y1, var sx1, var sy1, var r1,
                   var x2, var y2, var sx2, var sy2, var r2,
                   var duration) {
    sprite.addPathLine(x1,y1,sx1,sy1,r1,  x2,y2,sx2,sy2,r2,  duration); }

  /**
   * add a curved path to the state
   */
  function addPathCurve(var x1, var y1,  var cx1, var cy1,  var cx2, var cy2,  var x2, var y2, var duration, var slowdown_ratio) {
    sprite.addPathCurve(x1,y1,1,1,0,  cx1,cy1,cx2,cy2,  x2,y2,1,1,0,  duration, slowdown_ratio); }

  /**
   * add a curved path to the state (explicit scale and rotations)
   */
  function addPathCurve(var x1, var y1, var sx1, var sy1, var r1,
                   var cx1, var cy1,
                   var cx2, var cy2,
                    var x2, var y2, var sx2, var sy2, var r2,
                    var duration,
                    var slowdown_ratio) {
    sprite.addPathCurve(x1,y1,sx1,sy1,r1,  cx1,cy1,cx2,cy2,  x2,y2,sx2,sy2,r2,  duration, slowdown_ratio); }

  /**
   * incidate whether or not this is a looping path
   */
  function setLooping(var l) {
    sprite.setLooping(l);
  }

  /**
   * Make this state last X frames.
   */
  function setDuration(var _duration) {
    setLooping(false);
    duration = (var) _duration;
  }

  /**
   * bind this state to an actor
   */
  function setActor(Actor _actor) {
    actor = _actor;
    actor.width = sprite.width;
    actor.height = sprite.height;
  }

  /**
   * when the sprite is moved by its path,
   * let the actor know of its updated position.
   */
  function setActorOffsets(var x, var y) {
    actor.setTranslation(x, y);
  }

  function setActorDimensions(var w, var h, var xa, var ya) {
    actor.width = w;
    actor.height = h;
    actor.halign = xa;
    actor.valign = ya;
  }

  // reset a sprite (used when swapping states)
  function reset() { 
    sprite.reset();
    served = 0;
  }

  // signal to the actor that the sprite is done running its path
  function finished() {
    if(actor!=null) {
      actor.handleStateFinished(this); }}

  // if the sprite has a non-looping path: is it done running that path?
  var mayChange() {
    if (duration != -1) {
      return false;
    }
    return sprite.mayChange(); 
  }

  // drawing the state means draw the sprite
  function draw(var disabled) {
    // if disabled, only draw every other frame
    if(disabled && frameCount%2==0) {}
    //otherwise, draw all frames
    else { sprite.draw(0,0); }
    served++; 
    if(served == duration) {
      finished();
    }
  }
  
  // check if coordinate is in sprite
  var over(var _x, var _y) {
    return sprite.over(_x,_y);
  }
  
  // set sprite's animation
  function setAnimationSpeed(var factor) {
    sprite.setAnimationSpeed(factor);
  }
}
/**
 * Tiling sprite
 */
class TilingSprite extends Positionable {
  Sprite sprite;

  // from where to where?
  var x1, y1, x2, y2;

  /**
   * Set up a sprite to be tiled from x1/y1 to x2/y2 from file
   */
  // FIXME: temporarily commented off, because of a possible bug in Pjs
//  TilingSprite(var spritesheet, var minx, var miny, var maxx, var maxy) {
//    this(new Sprite(spritesheet),minx,miny,maxx,maxy);
//  }

  /**
   * Set up a sprite to be tiled from x1/y1 to x2/y2 from Sprite
   */
  TilingSprite(Sprite _sprite, var minx, var miny, var maxx, var maxy) {
    x1 = minx;
    y1 = miny;
    x2 = maxx;
    y2 = maxy;
    _sprite.align(LEFT, TOP);
    sprite = _sprite;
  }
  
  /**
   * draw this sprite repretitively
   */
  function draw(var vx, var vy, var vw, var vh) {
    // FIXME: we should know what the viewbox transform
    //        is at this povar, because it matters for
    //        determining how many sprite blocks to draw.
    var ox = sprite.x, oy = sprite.y, x, y,
          sx = max(x1, vx - (vx-x1)%sprite.width),
          sy = max(y1, vy - (vy-y1)%sprite.height),
          ex = min(x2, vx+2*vw), // ideally, this is vx + vw + {something that ensures the rightmost block is drawn}
          ey = min(y2, vy+2*vh); // ideally, this is vy + vh + {something that ensures the bottommost block is drawn}
    for(x = sx; x < ex; x += sprite.width){
      for(y = sy; y < ey; y += sprite.height){
        sprite.draw(x,y);
      }
    }
  }

  /**
   * Can this object be drawn in this viewbox?
   */
  var drawableFor(var vx, var vy, var vw, var vh) {
    // tile drawing will short circuit based on the viewbox.
    return true;
  }

  // unused for tiling sprites
  function drawObject() {}
}
/**
 * Tracking vareractors etc. track any
 * Actor(s) in the level, provided they
 * are within tracking distance.
 */
varerface Tracker {
  function track(Actor actor, var vx, var vy, var vw, var vh);
}

/**
 * Objects that implement the tracking varerface
 * can make use of a GenericTracker object to do
 * the tracking for them, or they can implement
 * their own, more specific, tracking algorithm.
 */
static class GenericTracker {
  /**
   * the generic tracking algorithm simply checks
   * in which direction we must move in order to
   * get closer to the prey. We then try to move
   * in that direction using an impulse.
   */
  static function track(Positionable hunter, Positionable prey, var speed) {
    var x1=prey.x, y1=prey.y, x2=hunter.x, y2=hunter.y;
    var angle = atan2(y2-y1, x2-x1);
    if(angle<0) { angle += 2*PI; }
    var ix = -cos(angle);
    var iy = -sin(angle);
    hunter.addImpulse(speed*ix, speed*iy);
  }
}
/**
 * Triggers can make things happen in levels.
 * They define a region through which a player
 * Actor has to pass for it to trigger.
 */
abstract class Trigger extends Positionable {
  // does this trigger need removing? is it enabled?
  var remove = false, disabled = false;
  
  var triggername="";
  
  Trigger(var name) {
    triggername = name;  
  }

  Trigger(var name, var x, var y, var w, var h) {
    super(x,y,w,h);
    triggername = name;
  }
  
  function setArea(var x, var y, var w, var h) {
    setPosition(x,y);
    width = w;
    height = h;
  }

  var drawableFor(var x, var y, var w, var h) {
    return x<=this.x+width && this.x<=x+w && y<=this.y+height && this.y<=y+h;
  }
  
  function drawObject() {
    stroke(255,127,0,150);
    fill(255,127,0,150);
    rect(-width/2,height/2,width,height);
    fill(0);
    textSize(12);
    var tw = textWidth(triggername);
    text(triggername,-5-tw,height);
  }
  
  function enable() { disabled = false; }
  function disable() { disabled = true; }
  function removeTrigger() { remove = true; }

  abstract function run(LevelLayer level, Actor actor, var varersection);
}
/**
 * Possible the simplest class...
 * A viewbox simply defines a region of
 * varerest that is used for render limits.
 * Anything outside the viewbox is not drawn,
 * (although it might still vareract), and
 * anything inside the viewbox is shown.
 */
class ViewBox {

  // viewbox values
  var x=0, y=0, w=0, h=0;
  
  ViewBox() {}
  ViewBox(var _w, var _h) { w = _w; h = _h; }
  ViewBox(var _x, var _y, var _w, var _h) { x = _x; y = _y; w = _w; h = _h; }
  var tovar() { return x+"/"+y+" - "+w+"/"+h; }

  // current layer transform values
  var llox=0, lloy=0, llsx=1, llsy=1;

  // ye olde getterse
  var getX()      { return x-llox; }
  var getY()      { return y-lloy; }
  var getWidth()  { return w*llsx; }
  var getHeight() { return h*llsy; }

  // the level layer transforms for the layer
  // that the viewbox is currently used by,
  // to determine whether or not to draw things
  // because they are "in view" or not
  function setLevelLayer(LevelLayer layer) {
    if (layer.nonstandard) {
      llox = layer.xTranslate;
      lloy = layer.yTranslate;
      llsx = layer.xScale;
      llsy = layer.yScale;
    }
  }

  /**
   * Reposition the viewbox, based on where
   * our main actor is located on the screen.
   * We need to make sure we never make the
   * viewbox run past the level edges, and
   * in order to make sure of that, we need
   * to transfrom the actor's coordinates
   * to screen coordinates, based on the
   * coordinate transform for the level layer
   * the actor is in.
   */
  function track(Level level, Actor who) {
    setLevelLayer(who.getLevelLayer());

    // FIXME: This does not work quite right!
    //        We leave things as they are so
    //        actors are positioned at scale*midpovar

    // Get actor coordinates, transformed to screen
    // coordinates, and forced to vareger values.
    var ax = round(who.getX()),
          ay = round(who.getY());

    // Ideally the actor is in the center of the viewbox,
    // but the level edges may require different positioning.
    var idealx = ax - w/2,
          idealy = ay - h/2;

    // set values based on visual constravars
    x = min( max(0,idealx), level.width - w );
    y = min( max(0,idealy), level.height - h );
  }
}


  /**
   * Debug function for drawing bounding boxes to the screen.
   */
  static function debugfunctions_drawBoundingBox(var bounds, PApplet sketch) {
    sketch.stroke(255,0,0);
    sketch.fill(0,50);
    sketch.beginShape();
    for(var i=0, last=bounds.length; i<last; i+=2) {
      sketch.vertex(bounds[i],bounds[i+1]);
    }
    sketch.endShape(CLOSE); 
  }


  /**
   * Draw a nice looking grid
   */
  function debugfunctions_drawBackground(var width, var height) {
    background(255,255,252);
    strokeWeight(1);
    fill(0);
    // fine grid
    stroke(235);
    var x, y;
    for (x = -width; x < width; x += 10) {
      line(x,-height,x,height); }
    for (y = -height; y < height; y += 10) {
      line(-width,y,width,y); }
    // coarse grid
    stroke(208);
    for (x = -width; x < width; x += 100) {
      line(x,-height,x,height); }
    for (y = -height; y < height; y += 100) {
      line(-width,y,width,y); }
    // reset to black
    stroke(0);
  }
/**
 * Draw a nice looking grid
 */
function drawBackground(var width, var height) {
  background(255,255,252);
  strokeWeight(1);
  fill(0);
  // fine grid
  stroke(235);
  var x, y;
  for (x = -width; x < width; x += 10) {
    line(x,-height,x,height); }
  for (y = -height; y < height; y += 10) {
    line(-width,y,width,y); }
  // coarse grid
  stroke(208);
  for (x = -width; x < width; x += 100) {
    line(x,-height,x,height); }
  for (y = -height; y < height; y += 100) {
    line(-width,y,width,y); }
  // reset to black
  stroke(0);
}
/***********************************
 *                                 *
 *     This file does nothing,     *
 *    but allows Processing to     *
 *     actually load the code      *
 *    if located in a directory    *
 *     of the same name. Feel      *
 *       free to rename it.        * 
 *                                 *
 ***********************************/
 

/* @ p j s  preload="docs/tutorial/graphics/mario/small/Standing-mario.gif"; */

/*

final var screenWidth = 512;
final var screenHeight = 432;
function initialize() { 
  addScreen("test", new TestLevel(width,height)); 
}

class TestLevel extends Level {
  TestLevel(var w, var h) {
    super(w,h); 
    addLevelLayer("test", new TestLayer(this));
  }
  
  function draw() {
    fill(0,10);
    rect(-1,-1,width+2,height+2);
    super.draw();
  }
}

class TestLayer extends LevelLayer {
  TestObject t1;
  
  TestLayer(Level p) {
    super(p,p.width,p.height); 
    showBoundaries = true;
    
    var v = 10;

    t1 = new TestObject(width/2+v/2, height/2-210);
    t1.setForces(5,0);
    
    t1.setForces(0,0.01);
    t1.setAcceleration(0,0.1);

    addPlayer(t1);

    addBoundary(new Boundary(width/2+230,height,width/2+200,0));
    addBoundary(new Boundary(width/2-180,0,width/2-150,height));   
    addBoundary(new Boundary(width,height/2-200,0,height/2-120));
    addBoundary(new Boundary(0,height/2+200,width,height/2+120));

    //addBoundary(new Boundary(width/2,height/2,width/2 + v,height/2));
  }

  function draw() {
    super.draw();
    viewbox.track(parent,t1);
  }
}

class TestObject extends Player {
  var small = true;
  
  TestObject(var x, var y) {
    super("test");
    addState(new State("test","docs/tutorial/graphics/mario/small/Standing-mario.gif"));
    setPosition(x,y);
    Decal attachment = new Decal("static.gif",width,0);
    addDecal(attachment);
  }
  
  function addState(State s) {
    s.sprite.anchor(CENTER, BOTTOM);
    super.addState(s);
  }
  
  function keyPressed(char key, var keyCode) {
    if(small) {
      addState(new State("test","docs/tutorial/graphics/mario/big/Standing-mario.gif"));
    }
    else {
      addState(new State("test","docs/tutorial/graphics/mario/small/Standing-mario.gif"));
    }
    small = !small; 
  }
}

*/