from threading import Timer, Lock

class FieldGetter(object):
    def __init__(self, field_name, monitor):
        self._field_name = field_name
        self._monitor = monitor
    def __call__(self):
        return self._monitor.get_field_value(self._field_name)


class StatusMonitor(object):
    def __init__(self, frequency):
        self._name="Status Monitor (BASE)"
        self._fields=self.get_fields()
        self._data = {}
        for i in self._fields:
            setattr(self,"_%s"%i,None)
            setattr(self, "get_%s"%i,FieldGetter(i,self))

        self._data_lock = Lock()
        self._frequency=frequency
        self._timer = Timer(0.01, self._on_timer)
        self._timer.start()
        self._conditions = []
        self._status="Ok"
        self._status_level = 0 # o=ok, 1=warn, 2=error

    def _on_timer(self):
        self._timer = Timer(1.0 / self._frequency, self._on_timer)
        self._timer.start()
        self._status="Ok"
        self._status_level=0
        self._data_lock.acquire()
        self.update()
        self._data_lock.release()

        # Check the warn conditions
        if self._status_level == 0:
            for c in self._conditions:
                if c[1](self.get_field_value(c[0])):
                    self._status = c[2]
                    self._status_level = 1
            
        
    def get_fields(self):
        return []

    def get_field_value(self, field):
        self._data_lock.acquire()
        val = getattr(self,"_%s"%field)
        self._data_lock.release()
        return val

    def get_name(self):
        return self._name

    def update(self):
        """ This better be faster than the timeout!"""
        pass

    def stop(self):
        self._timer.cancel()

    def add_warn_condition(self, field_name, condition, message):
        self._conditions.append([field_name, condition, message])

    def __str__(self):
        a="-"*10 + "\n"
        for i in self.get_fields():
            a+=i+":"+(20-len(i))*" "+str(getattr(self,"get_"+i)())+"\n"
        a+="-"*10
        return a
