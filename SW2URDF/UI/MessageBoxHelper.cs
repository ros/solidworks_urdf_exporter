
using System.Windows;

namespace SW2URDF.UI
{
    public class MessageBoxHelper : IMessageBox
    {
        MessageBoxResult IMessageBox.Show(string message)
        {
            return MessageBox.Show(message);
        }

        MessageBoxResult IMessageBox.Show(string message, string caption, MessageBoxButton buttons)
        {
            return MessageBox.Show(message, caption, buttons);
        }
    }
}
