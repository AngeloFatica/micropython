class Measurement:

    def __init__(self):
        # I think create_stream should be here... since everything depends on it.. but whatever for now
        pass

    def add_point(self, point):
        self.stream.append(round(point,self.stream_rounding))
        while len(self.stream) > self.stream_size:
            del self.stream[0]
            self.isfull = True

    def create_stream(self, size, rounding):
        self.stream = []
        self.stream_size = int(size)
        if self.stream_size == 0: self.stream_size = 1
        self.stream_rounding = int(rounding)
        self.isfull = False
        return()

    def average(self):
        try:
            self.avg = round(sum(self.stream)/len(self.stream),self.stream_rounding)
            return(self.avg)
        except Exception as e:
            print(e)
            pass

    def store_calibration_offset(self, offset):
        self.calibration_offset = offset

    def add_calibrated_point(self, point):
        if self.calibration_offset == None:
            self.store_calibration_offset(0)
        self.stream.append(round((point - self.calibration_offset),self.stream_rounding))
        if len(self.stream) > self.stream_size:
            del self.stream[0]

    def calibrate(self, desired_value):
        if len(self.stream) >= self.stream_size:
            self.cal_desired_value = desired_value
            self.cal_avg = self.average()
            self.cal_delta = self.cal_avg - self.cal_desired_value
            self.store_calibration_offset(self.cal_delta)

    def reset_stream(self):
        temp = self.stream[-1]
        self.stream.clear()
        self.stream.append(temp)
        self.isfull = False

    def check_slope(self):
        try:
            self.range = self.stream[-1]-self.stream[0]
            self.slope = self.range/len(self.stream)
            if abs(self.slope) > 2.5:
                print("Stream adjustment reducing lag", self.range, self.slope)
                self.reset_stream()
        except Exception as e:
            print(e)
            pass