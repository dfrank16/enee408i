from flask import Flask, render_template
from flask_ask import Ask, statement, question

app = Flask(__name__)
ask = Ask(app, '/')

@ask.launch
def launched():
 # monitorHR()
    return question("hello. what would you like Sparky to do?").reprompt(
        "if you don't need Sparky, please tell him to go to sleep.")


@ask.intent('AMAZON.FallbackIntent')
def default():
    return question("Sorry, Sparky doesn't understand that command. What would you like him to do?").reprompt(
        "What would you like Sparky to do now?")

@ask.intent('SleepIntent')
def sleep():
    # halt()
    print("WE sleepING boiis")
    return statement('Sparky says woof woof. Bye!')


if __name__ == '__main__':
    app.run(debug=True)