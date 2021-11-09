
module.exports = {
  prompt: ({ inquirer }) => {
    // defining questions in arrays ensures all questions are asked before next prompt is executed
    const questions = [{
      type: 'input',
      name: 'nodeName',
      message: 'What is the name of the node (camelCase)?',
    },
    {
      type: 'input',
      name: 'subscriptions',
      message: 'What topics should be subscribed to (comma seperated)?',
    },
    {
      type: 'input',
      name: 'publishers',
      message: 'What topics should be published (comma seperated)?',
    }]

    return inquirer
      .prompt(questions)
  },
}
