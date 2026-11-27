import math
def confirmed_position(retract,extend,powered):
    if powered is not True:raise ValueError('Confirm motor power on')
    retract=float(retract);extend=float(extend)
    if not all(math.isfinite(v) and 0<=v<=70 for v in (retract,extend)) or abs(retract+extend-70)>0.02:
        raise ValueError('Safe distances must sum to 70 mm')
    return round((retract-41.8)*160)

if __name__=='__main__':
    assert confirmed_position(41.8,28.2,True)==0
    assert confirmed_position(0,70,True)==-6688
    assert confirmed_position(70,0,True)==4512
    for args in ((41.8,28.2,False),(-1,71,True),(35,34,True),(float('nan'),70,True)):
        try:confirmed_position(*args)
        except ValueError:pass
        else:raise AssertionError('Invalid calibration accepted')
    print('PASS: safe-range conversion, both boundaries, power confirmation, invalid distances')
