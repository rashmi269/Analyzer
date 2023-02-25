import os
from flask import Flask, render_template, request, url_for, redirect
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.sql import func
import analyzer
app = Flask(__name__)   # Create an instance of flask called "app"




basedir = os.path.abspath(os.path.dirname(__file__))
# from testip import testip
# from push_data import push_data
# import RPi.GPIO as GPIO
# import time

# GPIO.setmode(GPIO.BCM)  # Sets up the RPi lib to use the Broadcom pin mappings
                        #  for the pin names. This corresponds to the pin names
                        #  given in most documentation of the Pi header
# GPIO.setwarnings(False) # Turn off warnings that may crop up if you have the
                        #  GPIO pins exported for use via command line
# GPIO.setup(2, GPIO.OUT) # Set GPIO2 as an output


app.config['SQLALCHEMY_DATABASE_URI'] =\
        'sqlite:///' + os.path.join(basedir, 'database.db')
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

db = SQLAlchemy(app)



class patient(db.Model):
    id = db.Column(db.Integer, unique=True, primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    gender = db.Column(db.String(80), nullable=False)
    age = db.Column(db.Integer,nullable=True)
    testname = db.Column(db.String(100), nullable=True)
    flag_level = db.Column(db.String(80), nullable=True)
    test_level = db.Column(db.String(80), nullable=True)
    date = db.Column(db.DateTime(timezone=True),server_default=func.now())

class new_test_visible(db.Model):
    test_name = db.Column(db.String(100), primary_key=True ,unique=True, nullable=True)
    test_type = db.Column(db.String(100), nullable=True)
    S = db.Column(db.Integer, nullable=True)
    H = db.Column(db.Integer, nullable=True)
    V = db.Column(db.Integer, nullable=True)
    L = db.Column(db.Integer, nullable=True)
    intercept = db.Column(db.Integer, nullable=True)
    test_temperature = db.Column(db.Integer, nullable=True)
    test_level_lower = db.Column(db.Integer, nullable=True)
    test_level_higher = db.Column(db.Integer, nullable=True)
    test_unit = db.Column(db.String(100), nullable=True)


class new_test_uv(db.Model):
    test_name = db.Column(db.String(100), primary_key=True ,unique=True, nullable=True)
    test_type = db.Column(db.String(100), nullable=True)
    test_stdval = db.Column(db.Integer,nullable=True)
    test_temperature = db.Column(db.Integer, nullable=True)
    test_level_lower = db.Column(db.Integer, nullable=True)
    test_level_higher = db.Column(db.Integer, nullable=True)
    test_unit = db.Column(db.String(100), nullable=True)



@app.route("/")
def index():
    return render_template("index.html"), {"Refresh": "5; url=list_of_biochemistry"}

@app.route("/list_of_biochemistry")
def list_of_biochemistry():
    tests_visible = new_test_visible.query.all()
    tests_uv = new_test_uv.query.all()

    return render_template("list_of_biochemistry.html", tests_visible = tests_visible,tests_uv = tests_uv )



@app.route("/add_new_test_visible",methods=['GET','POST'])
def add_new_test_visible():
    if request.method == 'POST':

        test_name = request.form['testname']
        test_type = request.form['testtype']
        H = request.form['H']
        S = request.form['S']
        L = request.form['L']
        V = request.form['V']
        test_temperature = request.form['test_temperature']
        intercept = request.form['intercept']
        test_level_lower = request.form['test_level_lower']
        test_level_higher = request.form['test_level_higher']
        test_unit = request.form['test_level_higher']

        test = new_test_visible( 
                    test_name = test_name, 
                    test_type = test_type ,
                    S=S,H=H,V=V,L=L,
                    intercept = intercept,
                    test_temperature = test_temperature,
                    test_level_lower = test_level_lower, 
                    test_level_higher = test_level_higher,
                    test_unit = test_unit )

        db.session.add(test)
        db.session.commit()

    return render_template("new_test_added.html"),{"Refresh": "2; url=list_of_biochemistry"}

@app.route("/add_new_test_uv",methods=['GET','POST'])
def add_new_test_uv():
    if request.method == 'POST':

        test_name = request.form['testname']
        test_type = request.form['testtype']
        test_level_lower = request.form['test_level_lower']
        test_level_higher = request.form['test_level_higher']
        test_unit = request.form['test_unit']
        test_temperature = request.form['test_temperature']

        test = new_test_uv( 
                    test_name = test_name, 
                    test_type = test_type ,
                    test_temperature = test_temperature,
                    test_level_lower = test_level_lower, 
                    test_level_higher = test_level_higher,
                    test_unit = test_unit 
                    )

        
        db.session.add(test)
        db.session.commit()

    return render_template("new_test_added.html"),{"Refresh": "2; url=list_of_biochemistry"}


@app.route("/test_done",methods=['GET','POST'])
def test_done():
    visible_list = []
    uv_list=[]

    if request.method == "POST":
        test_list = request.form.getlist('test_list')
       
        for test in test_list:
            current_test = test.split(",")
            if current_test[1] == "uv":
                test_details = new_test_uv.query.get_or_404(current_test[0])
                flag_low = test_details.test_level_lower
                flag_high = test_details.test_level_higher
                temperature = test_details.test_temperature
                # uv =  analyzer.UV()
                # value,flag,absorbance =    uv.uv_spectrum(flag_low,flag_high,temperature)
                # uv_list.append(value)
                # uv_list.append(flag)
                # uv_list.append(absorbance)
                uv_list.append(10)
                uv_list.append("low")
                uv_list.append(24)



            elif current_test[1] == "visible":
                test_details = new_test_visible.query.get_or_404(current_test[0])
                h = test_details.H
                s = test_details.S
                l = test_details.L
                v = test_details.V
                intercept = test_details.intercept
                flag_low = test_details.test_level_lower
                flag_high = test_details.test_level_higher
                temperature = test_details.test_temperature
                # value,flag = analyzer.visible_spectrum(h,s,l,v,intercept,flag_low,flag_high,temperature)
                # visible_list.append(value)
                # visible_list.append(flag)
                visible_list.append(19 )
                visible_list.append("high")
                

            # x_axis = []
            # y_axis = []
            # for i in range(len(int(absorbance+1))):
            #     x_axis.append(i)
            # for i in range(len(int(absorbance+1))):
            #     x_axis.append(i)
            
    return render_template("test_done.html",uv_list = uv_list,visible_list = visible_list)





@app.route("/delete_test",methods=['GET','POST'])
def delete_test():
    if request.method == "POST":
        test_list = request.form.getlist('test_list')
        for test in test_list:
            current_test = test.split(",")
           
            if current_test[1] == "uv":
                test_details = new_test_uv.query.get_or_404(current_test[0])

            else:
                test_details = new_test_visible.query.get_or_404(current_test[0])
        
        
            db.session.delete(test_details)
            db.session.commit()

    

    return redirect("/list_of_biochemistry")


@app.route("/edit_test",methods=['GET','POST'])
def edit_test():
    if request.method == "POST":
        test_list = request.form.getlist('test_list')
        edit_test_uv = dict(test_name = "", test_type = " ",flag_low = 0,flag_high=0,temperature = 0,test_unit = "")
        edit_test_visible = dict(test_name = "", test_type = " ",h="",s="",l="",v="",intercept=0,flag_low = 0,flag_high=0,temperature = 0,test_unit = "")
        for test in test_list:
            current_test = test.split(",")
            
            if current_test[1] == "uv":
                test_details = new_test_uv.query.get_or_404(current_test[0])
                test_name = test_details.test_name
                testtype = test_details.test_type
                flag_low = test_details.test_level_lower
                flag_high = test_details.test_level_higher
                temperature = test_details.test_temperature
                test_unit = test_details.test_unit

                edit_test_uv["test_name"] = test_name
                edit_test_uv["test_type"] = testtype
                edit_test_uv["flag_low"] = flag_low
                edit_test_uv["flag_high"] = flag_high
                edit_test_uv["temperature"] = temperature
                edit_test_uv["test_unit"] = test_unit

            else:
                test_details = new_test_visible.query.get_or_404(current_test[0])
                test_name = test_details.test_name
                testtype = test_details.test_type
                h = test_details.H
                s = test_details.S
                l = test_details.L
                v = test_details.V
                intercept = test_details.intercept
                flag_low = test_details.test_level_lower
                flag_high = test_details.test_level_higher
                temperature = test_details.test_temperature
                test_unit = test_details.test_unit

                edit_test_visible["test_name"] = test_name
                edit_test_visible["test_type"] = testtype
                edit_test_visible["h"] = h
                edit_test_visible["s"] = s
                edit_test_visible["l"] = l
                edit_test_visible["v"] = v
                edit_test_visible["intercept"] = intercept
                edit_test_visible["flag_low"] = flag_low
                edit_test_visible["flag_high"] = flag_high
                edit_test_visible["temperature"] = temperature
                edit_test_uv["test_unit"] = test_unit
    return render_template("edit_test.html",edit_test_uv = edit_test_uv, edit_test_visible = edit_test_visible)

@app.route("/update_test",methods=['GET','POST'])
def update_test():
    if request.method == "POST":
        test_type = request.form['testtype']
        print(test_type)
        if(test_type=="visible"):
            test_name = request.form['testnameold']
            test_details = new_test_visible.query.get_or_404(test_name)
            test_details.test_name = request.form['testname']
            test_details.H = request.form['H']
            test_details.S = request.form['S']
            test_details.L = request.form['L']
            test_details.V = request.form['V']
            test_details.test_temperature = request.form['test_temperature']
            test_details.intercept = request.form['intercept']
            test_details.test_level_lower = request.form['test_level_lower']
            test_details.test_level_higher = request.form['test_level_higher']
            test_details.test_unit = request.form['test_unit']
            db.session.add(test_details)
            db.session.commit()
        
        if test_type == "uv":
            test_name = request.form['testnameold']
            test_details = new_test_uv.query.get_or_404(test_name)
            test_details.test_name = request.form['testname']
            test_details.test_temperature = request.form['test_temperature']
            test_details.test_level_lower = request.form['test_level_lower']
            test_details.test_level_higher = request.form['test_level_higher']
            test_details.test_unit = request.form['test_unit']
            db.session.add(test_details)
            db.session.commit()



        return redirect("/list_of_biochemistry")

if __name__ == '__main__':
   app.run(debug = True)



if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)