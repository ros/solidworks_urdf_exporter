
using System.Windows;

namespace SW2URDF.UI
{
    public class MessageBoxHelper : IMessageBox
    {
        public MessageBoxResult Show(string message)
        {
            return MessageBox.Show(message);
        }

        public MessageBoxResult Show(string message, string caption, MessageBoxButton buttons)
        {
            return MessageBox.Show(message, caption, buttons);
        }
    }
}
